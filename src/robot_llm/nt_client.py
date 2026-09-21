"""NetworkTables client for the robot-side ``LlmCommands`` protocol.

The robot publishes a JSON *manifest* under ``/LLM/manifest`` describing each command it
exposes. For every command there is a sub-table ``/LLM/commands/<name>`` containing:

``params/<param>``   written by us before triggering
``run``              we set ``True``; the robot resets it once consumed
``cancel``           we set ``True`` to cancel a running instance
``status``           ``idle`` | ``running`` | ``finished`` | ``interrupted`` | ``aborted`` |
                     ``rejected``
``completedCount``   incremented by the robot every time a run ends
``lastResult``       human-readable outcome of the last run
``expected``         JSON object of the end values the run should reach, keyed by state key

Telemetry lives under ``/LLM/state``. While a command runs, :meth:`RobotClient.run_command`
samples the state keys the manifest lists as ``trackedState`` into a
:class:`~robot_llm.monitor.CommandTrace` and evaluates client-side rules against it. The Java
counterpart is ``ExampleRobotProject/src/main/java/frc/robot/llm/LlmCommands.java``.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any

import ntcore

from robot_llm.monitor import DEFAULT_RULES, CommandTrace, Rule, stalled_for

TABLE_NAME = "LLM"
DEFAULT_POLL_SECONDS = 0.02
# How often the tracked state is recorded into the trace while a command runs.
SAMPLE_SECONDS = 0.1
# Default check-in budget: how long a tool blocks before handing the trace back to the model.
DEFAULT_CHECK_IN_SECONDS = 5.0
# After the client cancels a command, how long to wait for the robot to acknowledge.
CANCEL_ACK_SECONDS = 1.0


@dataclass(frozen=True)
class ParamSpec:
    name: str
    type: str  # double | integer | boolean | string
    description: str = ""
    min: float | None = None
    max: float | None = None
    choices: list[str] | None = None

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> ParamSpec:
        return cls(
            name=data["name"],
            type=data.get("type", "string"),
            description=data.get("description", ""),
            min=data.get("min"),
            max=data.get("max"),
            choices=data.get("choices"),
        )


@dataclass(frozen=True)
class CommandSpec:
    name: str
    description: str = ""
    timeout_seconds: float = 0.0
    parameters: list[ParamSpec] = field(default_factory=list)
    # Monitoring hints published by the robot (see LlmCommandBuilder.track/checkIn/...).
    tracked_state: list[str] = field(default_factory=list)
    check_in_seconds: float = 0.0
    stall_timeout_seconds: float = 0.0
    has_watchdog: bool = False

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> CommandSpec:
        return cls(
            name=data["name"],
            description=data.get("description", ""),
            timeout_seconds=float(data.get("timeoutSeconds", 0.0)),
            parameters=[ParamSpec.from_json(p) for p in data.get("parameters", [])],
            tracked_state=list(data.get("trackedState", [])),
            check_in_seconds=float(data.get("checkInSeconds", 0.0)),
            stall_timeout_seconds=float(data.get("stallTimeoutSeconds", 0.0)),
            has_watchdog=bool(data.get("hasWatchdog", False)),
        )


@dataclass(frozen=True)
class Manifest:
    version: int
    commands: list[CommandSpec]

    @classmethod
    def from_json(cls, text: str) -> Manifest:
        data = json.loads(text)
        return cls(
            version=int(data.get("version", 0)),
            commands=[CommandSpec.from_json(c) for c in data.get("commands", [])],
        )

    def get(self, name: str) -> CommandSpec | None:
        return next((c for c in self.commands if c.name == name), None)


@dataclass(frozen=True)
class CommandStatus:
    """Live status of one command, read from ``/LLM/commands/<name>``."""

    name: str
    status: str  # idle | running | finished | interrupted | aborted | rejected | unknown
    last_result: str
    completed_count: int


@dataclass(frozen=True)
class CommandResult:
    """Outcome of one ``run_command`` / ``wait_for_command`` call.

    ``status`` is the robot's status once the run ended, or ``running`` when the check-in budget
    elapsed first, ``aborted`` when a client rule cancelled it, ``timeout`` when the client gave
    up, ``disconnected`` when the NT connection dropped.
    """

    name: str
    status: str
    message: str
    elapsed_seconds: float
    trace: CommandTrace | None = None

    @property
    def ok(self) -> bool:
        return self.status == "finished" and not (self.trace and self.trace.discrepancies)

    def __str__(self) -> str:
        return f"{self.name}: {self.status} ({self.message}) after {self.elapsed_seconds:.1f}s"


class RobotClient:
    """Thin wrapper around an ntcore client instance implementing the command protocol."""

    def __init__(
        self,
        *,
        server: str | None = None,
        team: int | None = None,
        port: int = ntcore.NetworkTableInstance.kDefaultPort4,
        identity: str = "robot-llm-client",
        default_timeout: float = 30.0,
    ) -> None:
        if server is None and team is None:
            server = "localhost"

        self._inst = ntcore.NetworkTableInstance.getDefault()
        self._inst.startClient4(identity)
        if team is not None:
            self._inst.setServerTeam(team, port)
        else:
            self._inst.setServer(server, port)

        self._table = self._inst.getTable(TABLE_NAME)
        self._commands_table = self._table.getSubTable("commands")
        self._state_table = self._table.getSubTable("state")
        # NT4 only delivers topics we subscribe to; state keys are discovered dynamically, so
        # subscribe to the whole prefix rather than individual entries.
        self._state_sub = ntcore.MultiSubscriber(self._inst, [f"/{TABLE_NAME}/state/"])
        self._manifest_entry = self._table.getEntry("manifest")
        self._cancel_all_entry = self._table.getEntry("cancelAll")
        self.default_timeout = default_timeout
        self.check_in_seconds = DEFAULT_CHECK_IN_SECONDS
        self.rules: list[Rule] = list(DEFAULT_RULES)
        self._manifest: Manifest | None = None
        self._active: dict[str, _ActiveRun] = {}
        self._traces: dict[str, CommandTrace] = {}

    # ------------------------------------------------------------------ connection

    @property
    def connected(self) -> bool:
        return self._inst.isConnected()

    def wait_for_connection(self, timeout: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.connected:
                return True
            time.sleep(0.1)
        return self.connected

    def close(self) -> None:
        self._inst.stopClient()

    # ------------------------------------------------------------------ manifest

    def get_manifest(self, timeout: float = 5.0, refresh: bool = False) -> Manifest:
        """Return the robot's command manifest, waiting up to ``timeout`` for it to appear."""
        if self._manifest is not None and not refresh:
            return self._manifest

        deadline = time.monotonic() + timeout
        text = self._manifest_entry.getString("")
        while not text and time.monotonic() < deadline:
            time.sleep(0.1)
            text = self._manifest_entry.getString("")
        if not text:
            raise TimeoutError(
                "Robot did not publish /LLM/manifest. Is the robot code running and does it call "
                "LlmCommands.getInstance().periodic()?"
            )
        self._manifest = Manifest.from_json(text)
        return self._manifest

    # ------------------------------------------------------------------ commands

    def run_command(
        self,
        name: str,
        args: dict[str, Any] | None = None,
        *,
        wait: bool = True,
        timeout: float | None = None,
        check_in: float | None = None,
    ) -> CommandResult:
        """Trigger ``name`` with ``args`` and (optionally) block until it finishes.

        While waiting, the tracked state keys are sampled into a trace and the client-side rules
        are evaluated; a rule that fires cancels the command. If the command is still running
        after ``check_in`` seconds (default: the manifest's ``checkInSeconds``, else
        :attr:`check_in_seconds`) the call returns with status ``running`` so the caller can look
        at the trace and decide whether to :meth:`wait_for_command` or :meth:`cancel_command`.
        """
        spec = self.get_manifest().get(name)
        if spec is None:
            raise KeyError(f"Unknown robot command: {name!r}")
        args = args or {}

        if name in self._active:
            # Re-triggering restarts the robot-side command; drop the stale trace.
            self._active.pop(name)

        table = self._commands_table.getSubTable(name)
        params_table = table.getSubTable("params")
        for param in spec.parameters:
            if param.name not in args:
                continue
            self._write_param(params_table.getEntry(param.name), param, args[param.name])

        # Robot-side timeout plus slack; the caller's timeout wins if smaller.
        if timeout is None:
            timeout = self.default_timeout
            if spec.timeout_seconds > 0:
                timeout = min(timeout, spec.timeout_seconds + 2.0)

        state = self.get_state()
        trace = CommandTrace(
            name=name,
            args=dict(args),
            tracked=list(spec.tracked_state),
            start_state={k: state.get(k) for k in spec.tracked_state},
        )
        run = _ActiveRun(
            spec=spec,
            trace=trace,
            start_count=table.getEntry("completedCount").getInteger(0),
            started=time.monotonic(),
            deadline=time.monotonic() + timeout,
            rules=self._rules_for(spec),
        )

        self._inst.flush()
        table.getEntry("run").setBoolean(True)
        self._inst.flush()

        if not wait:
            self._active[name] = run
            return CommandResult(name, "scheduled", "run request sent", 0.0, trace)

        return self._wait(run, check_in)

    def wait_for_command(self, name: str, check_in: float | None = None) -> CommandResult:
        """Keep waiting on a run that returned ``running`` (or ``scheduled``) earlier."""
        run = self._active.get(name)
        if run is None:
            trace = self._traces.get(name)
            if trace is not None:
                return CommandResult(name, trace.status, trace.message, trace.elapsed_seconds, trace)
            return CommandResult(name, "idle", "no run of this command is in progress", 0.0)
        return self._wait(run, check_in)

    def get_trace(self, name: str) -> CommandTrace | None:
        """The trace of the current or most recent run of ``name``."""
        run = self._active.get(name)
        return run.trace if run is not None else self._traces.get(name)

    def _rules_for(self, spec: CommandSpec) -> list[Rule]:
        rules = list(self.rules)
        if spec.stall_timeout_seconds > 0 and spec.tracked_state:
            # Backstop for the robot's own stall check, at twice its window so it fires first.
            rules.append(stalled_for(2.0 * spec.stall_timeout_seconds))
        return rules

    def _wait(self, run: _ActiveRun, check_in: float | None) -> CommandResult:
        name = run.spec.name
        trace = run.trace
        table = self._commands_table.getSubTable(name)
        completed_entry = table.getEntry("completedCount")
        status_entry = table.getEntry("status")
        result_entry = table.getEntry("lastResult")
        expected_entry = table.getEntry("expected")

        if check_in is None:
            check_in = run.spec.check_in_seconds or self.check_in_seconds
        budget_end = time.monotonic() + check_in
        self._active[name] = run

        while True:
            now = time.monotonic()
            elapsed = now - run.started

            if completed_entry.getInteger(0) > run.start_count:
                self._read_expected(run, expected_entry)
                status = status_entry.getString("unknown")
                message = result_entry.getString("")
                if status == "aborted":
                    trace.add_violation(elapsed, "robot", message)
                return self._finish(run, status, message)

            if not self.connected:
                return self._finish(run, "disconnected", "lost connection to robot")

            if now >= run.deadline:
                return self._finish(
                    run,
                    "timeout",
                    f"client stopped waiting after {run.deadline - run.started:.0f}s; robot status "
                    f"is {status_entry.getString('unknown')!r}",
                )

            if now - run.last_sample >= SAMPLE_SECONDS:
                run.last_sample = now
                state = self.get_state()
                acknowledged = status_entry.getString("") == "running"
                if acknowledged:
                    self._read_expected(run, expected_entry)
                if trace.tracked:
                    trace.add_sample(elapsed, state)
                if acknowledged:
                    reason = self._check_rules(run, state)
                    if reason is not None:
                        trace.add_violation(elapsed, "client", reason)
                        self.cancel_command(name)
                        self._await_cancel(run, completed_entry)
                        return self._finish(run, "aborted", f"client monitor: {reason}")

            if now >= budget_end:
                trace.elapsed_seconds = round(elapsed, 2)
                trace.status = "running"
                trace.message = f"still running after {elapsed:.1f}s"
                return CommandResult(name, "running", trace.message, elapsed, trace)

            time.sleep(DEFAULT_POLL_SECONDS)

    def _check_rules(self, run: _ActiveRun, state: dict[str, Any]) -> str | None:
        for rule in run.rules:
            reason = rule(run.trace, state)
            if reason:
                return reason
        return None

    def _await_cancel(self, run: _ActiveRun, completed_entry: ntcore.NetworkTableEntry) -> None:
        deadline = time.monotonic() + CANCEL_ACK_SECONDS
        while time.monotonic() < deadline and completed_entry.getInteger(0) <= run.start_count:
            time.sleep(DEFAULT_POLL_SECONDS)

    def _read_expected(self, run: _ActiveRun, entry: ntcore.NetworkTableEntry) -> None:
        if run.expected_read:
            return
        text = entry.getString("")
        if not text:
            return
        try:
            data = json.loads(text)
        except ValueError:
            return
        if isinstance(data, dict):
            run.trace.expected = {k: _round(v) for k, v in data.items()}
        run.expected_read = True

    def _finish(self, run: _ActiveRun, status: str, message: str) -> CommandResult:
        trace = run.trace
        elapsed = time.monotonic() - run.started
        trace.status = status
        trace.message = message
        trace.elapsed_seconds = round(elapsed, 2)
        state = self.get_state()
        trace.end_state = {k: state.get(k) for k in set(trace.tracked) | set(trace.expected)}
        if trace.tracked:
            trace.add_sample(elapsed, state)
        if status == "finished":
            trace.compute_discrepancies()
        self._active.pop(run.spec.name, None)
        self._traces[run.spec.name] = trace
        return CommandResult(run.spec.name, status, message, elapsed, trace)

    def cancel_command(self, name: str) -> None:
        self._commands_table.getSubTable(name).getEntry("cancel").setBoolean(True)
        self._inst.flush()

    def cancel_all(self) -> None:
        self._cancel_all_entry.setBoolean(True)
        self._inst.flush()

    def get_status(self, name: str) -> str:
        return self._commands_table.getSubTable(name).getEntry("status").getString("unknown")

    def get_command_status(self, name: str) -> CommandStatus:
        table = self._commands_table.getSubTable(name)
        return CommandStatus(
            name=name,
            status=table.getEntry("status").getString("unknown"),
            last_result=table.getEntry("lastResult").getString(""),
            completed_count=table.getEntry("completedCount").getInteger(0),
        )

    def get_all_statuses(self) -> list[CommandStatus]:
        """Status of every command in the manifest (empty if the manifest is not known yet)."""
        if self._manifest is None:
            return []
        return [self.get_command_status(spec.name) for spec in self._manifest.commands]

    # ------------------------------------------------------------------ state

    def get_state(self) -> dict[str, Any]:
        """Return every value under ``/LLM/state`` as a flat ``{"a/b": value}`` dict."""
        return self._read_table(self._state_table, prefix="")

    def _read_table(self, table: ntcore.NetworkTable, prefix: str) -> dict[str, Any]:
        out: dict[str, Any] = {}
        for key in sorted(table.getKeys()):
            value = table.getEntry(key).getValue()
            out[prefix + key] = _round(value.value())
        for sub in sorted(table.getSubTables()):
            out.update(self._read_table(table.getSubTable(sub), prefix=f"{prefix}{sub}/"))
        return out

    # ------------------------------------------------------------------ helpers

    @staticmethod
    def _write_param(entry: ntcore.NetworkTableEntry, param: ParamSpec, value: Any) -> None:
        match param.type:
            case "double":
                entry.setDouble(float(value))
            case "integer":
                entry.setInteger(int(value))
            case "boolean":
                entry.setBoolean(bool(value))
            case _:
                entry.setString(str(value))


def _round(value: Any) -> Any:
    if isinstance(value, float):
        return round(value, 3)
    return value


@dataclass
class _ActiveRun:
    """Bookkeeping for a run that has been triggered and not yet finished."""

    spec: CommandSpec
    trace: CommandTrace
    start_count: int
    started: float
    deadline: float
    rules: list[Rule]
    last_sample: float = 0.0
    expected_read: bool = False
