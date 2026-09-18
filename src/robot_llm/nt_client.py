"""NetworkTables client for the robot-side ``LlmCommands`` protocol.

The robot publishes a JSON *manifest* under ``/LLM/manifest`` describing each command it
exposes. For every command there is a sub-table ``/LLM/commands/<name>`` containing:

``params/<param>``   written by us before triggering
``run``              we set ``True``; the robot resets it once consumed
``cancel``           we set ``True`` to cancel a running instance
``status``           ``idle`` | ``running`` | ``finished`` | ``interrupted`` | ``rejected``
``completedCount``   incremented by the robot every time a run ends
``lastResult``       human-readable outcome of the last run

Telemetry lives under ``/LLM/state``. The Java counterpart is
``ExampleRobotProject/src/main/java/frc/robot/llm/LlmCommands.java``.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any

import ntcore

TABLE_NAME = "LLM"
DEFAULT_POLL_SECONDS = 0.02


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

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> CommandSpec:
        return cls(
            name=data["name"],
            description=data.get("description", ""),
            timeout_seconds=float(data.get("timeoutSeconds", 0.0)),
            parameters=[ParamSpec.from_json(p) for p in data.get("parameters", [])],
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
class CommandResult:
    name: str
    status: str
    message: str
    elapsed_seconds: float

    @property
    def ok(self) -> bool:
        return self.status == "finished"

    def __str__(self) -> str:
        return f"{self.name}: {self.status} ({self.message}) after {self.elapsed_seconds:.1f}s"


class RobotClient:
    """Thin wrapper around an ntcore client instance implementing the command protocol."""

    def __init__(
        self,
        *,
        server: str | None = None,
        team: int | None = None,
        identity: str = "robot-llm-client",
        default_timeout: float = 30.0,
    ) -> None:
        if server is None and team is None:
            server = "localhost"

        self._inst = ntcore.NetworkTableInstance.getDefault()
        self._inst.startClient4(identity)
        if team is not None:
            self._inst.setServerTeam(team)
        else:
            self._inst.setServer(server)

        self._table = self._inst.getTable(TABLE_NAME)
        self._commands_table = self._table.getSubTable("commands")
        self._state_table = self._table.getSubTable("state")
        # NT4 only delivers topics we subscribe to; state keys are discovered dynamically, so
        # subscribe to the whole prefix rather than individual entries.
        self._state_sub = ntcore.MultiSubscriber(self._inst, [f"/{TABLE_NAME}/state/"])
        self._manifest_entry = self._table.getEntry("manifest")
        self._cancel_all_entry = self._table.getEntry("cancelAll")
        self.default_timeout = default_timeout
        self._manifest: Manifest | None = None

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
    ) -> CommandResult:
        """Trigger ``name`` with ``args`` and (optionally) block until it finishes."""
        spec = self.get_manifest().get(name)
        if spec is None:
            raise KeyError(f"Unknown robot command: {name!r}")
        args = args or {}

        table = self._commands_table.getSubTable(name)
        params_table = table.getSubTable("params")
        for param in spec.parameters:
            if param.name not in args:
                continue
            self._write_param(params_table.getEntry(param.name), param, args[param.name])

        completed_entry = table.getEntry("completedCount")
        status_entry = table.getEntry("status")
        result_entry = table.getEntry("lastResult")
        run_entry = table.getEntry("run")

        start_count = completed_entry.getInteger(0)
        started = time.monotonic()
        self._inst.flush()
        run_entry.setBoolean(True)
        self._inst.flush()

        if not wait:
            return CommandResult(name, "scheduled", "run request sent", 0.0)

        # Robot-side timeout plus slack; the caller's timeout wins if smaller.
        if timeout is None:
            timeout = self.default_timeout
            if spec.timeout_seconds > 0:
                timeout = min(timeout, spec.timeout_seconds + 2.0)

        deadline = started + timeout
        while time.monotonic() < deadline:
            if completed_entry.getInteger(0) > start_count:
                return CommandResult(
                    name,
                    status_entry.getString("unknown"),
                    result_entry.getString(""),
                    time.monotonic() - started,
                )
            if not self.connected:
                return CommandResult(
                    name, "disconnected", "lost connection to robot", time.monotonic() - started
                )
            time.sleep(DEFAULT_POLL_SECONDS)

        return CommandResult(
            name,
            "timeout",
            f"client stopped waiting after {timeout:.0f}s; robot status is "
            f"{status_entry.getString('unknown')!r}",
            time.monotonic() - started,
        )

    def cancel_command(self, name: str) -> None:
        self._commands_table.getSubTable(name).getEntry("cancel").setBoolean(True)
        self._inst.flush()

    def cancel_all(self) -> None:
        self._cancel_all_entry.setBoolean(True)
        self._inst.flush()

    def get_status(self, name: str) -> str:
        return self._commands_table.getSubTable(name).getEntry("status").getString("unknown")

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
