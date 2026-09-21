"""Integration tests: ``RobotClient`` against an in-process NetworkTables server that plays the
robot, implementing the same ``/LLM`` protocol as ``LlmCommands.java``."""

from __future__ import annotations

import json
import socket
import threading
import time

import ntcore
import pytest

from robot_llm.nt_client import RobotClient

X, Y, H = "drivetrain/x_meters", "drivetrain/y_meters", "drivetrain/heading_degrees"
LOOP = 0.02
SPEED = 2.0  # m/s, so a 1 m drive takes ~0.5 s of wall time

MANIFEST = {
    "version": 3,
    "table": "/LLM",
    "commands": [
        {
            "name": "drive_distance",
            "description": "drive",
            "timeoutSeconds": 10.0,
            "checkInSeconds": 0.0,
            "stallTimeoutSeconds": 0.0,
            "hasWatchdog": True,
            "trackedState": [X, Y, H],
            "parameters": [{"name": "meters", "type": "double", "description": "m"}],
        },
        {"name": "idle", "description": "never finishes", "timeoutSeconds": 0.0, "parameters": []},
        {
            "name": "self_abort",
            "description": "robot watchdog fires",
            "timeoutSeconds": 5.0,
            "trackedState": [X],
            "parameters": [],
        },
    ],
}


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class FakeRobot:
    """Minimal stand-in for the Java registry: one active command, kinematic drivetrain."""

    def __init__(self, port4: int, persist: str) -> None:
        self.inst = ntcore.NetworkTableInstance.create()
        self.inst.startServer(persist, "127.0.0.1", _free_port(), port4)
        table = self.inst.getTable("LLM")
        table.getEntry("manifest").setString(json.dumps(MANIFEST))
        self.commands = table.getSubTable("commands")
        self.state = table.getSubTable("state")
        self.x = self.y = self.heading = 0.0
        self.drift = 0.0  # m/s sideways drift while driving, to provoke the client rule
        self.short_by = 0.0  # finish this many meters early, to provoke a discrepancy
        self.active: str | None = None
        self.target = 0.0
        self.started = 0.0
        self.runs_started = 0
        self.ignore_requests = False  # play dead, to provoke the client's ack timeout
        # Last request id seen per command/kind; None until the topic first appears.
        self.last_request: dict[tuple[str, str], int | None] = {}
        self._stop = threading.Event()
        for cmd in MANIFEST["commands"]:
            t = self.commands.getSubTable(cmd["name"])
            t.getEntry("runAck").setInteger(0)
            t.getEntry("status").setString("idle")
            t.getEntry("completedCount").setInteger(0)
            t.getEntry("lastResult").setString("")
            t.getEntry("expected").setString("{}")
            # Like the Java registry, publish a default so the param topic exists up front.
            for param in cmd["parameters"]:
                t.getSubTable("params").getEntry(param["name"]).setDefaultDouble(0.0)
        self.publish_state()
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def close(self) -> None:
        self._stop.set()
        self.thread.join(2.0)
        self.inst.stopServer()
        ntcore.NetworkTableInstance.destroy(self.inst)

    def publish_state(self) -> None:
        self.state.getEntry(X).setDouble(self.x)
        self.state.getEntry(Y).setDouble(self.y)
        self.state.getEntry(H).setDouble(self.heading)
        self.state.getEntry("robot/enabled").setBoolean(True)
        self.state.getEntry("avoidance_zones").setString("[]")

    def finish(self, status: str, result: str) -> None:
        t = self.commands.getSubTable(self.active)
        t.getEntry("status").setString(status)
        t.getEntry("lastResult").setString(result)
        t.getEntry("completedCount").setInteger(t.getEntry("completedCount").getInteger(0) + 1)
        self.active = None

    def new_request(self, name: str, kind: str) -> bool:
        """Same rule as LlmCommands.RequestCounter: once per id larger than the last seen."""
        entry = self.commands.getSubTable(name).getEntry(kind)
        if not entry.exists():
            return False
        value = entry.getInteger(0)
        last = self.last_request.get((name, kind))
        self.last_request[(name, kind)] = value
        return last is not None and value > last

    def start(self, name: str) -> None:
        t = self.commands.getSubTable(name)
        self.runs_started += 1
        expected: dict[str, object] = {}
        if name == "drive_distance":
            self.target = t.getSubTable("params").getEntry("meters").getDouble(0.0)
            expected = {X: self.x + self.target, Y: self.y, H: self.heading}
        t.getEntry("expected").setString(json.dumps(expected))
        t.getEntry("status").setString("running")
        self.active = name
        self.started = time.monotonic()
        self.start_x = self.x

    def loop(self) -> None:
        while not self._stop.is_set():
            for cmd in MANIFEST["commands"]:
                name = cmd["name"]
                if self.new_request(name, "cancelRequest") and self.active == name:
                    self.finish("interrupted", "command was cancelled")
                if self.new_request(name, "runRequest") and not self.ignore_requests:
                    self.commands.getSubTable(name).getEntry("runAck").setInteger(
                        self.last_request[(name, "runRequest")]
                    )
                    if self.active == name:
                        self.finish("interrupted", "command was cancelled")
                    self.start(name)
            if self.active == "drive_distance":
                self.x += SPEED * LOOP
                self.y += self.drift * LOOP
                if self.x - self.start_x >= self.target - self.short_by:
                    self.finish("finished", "command completed")
            elif self.active == "self_abort" and time.monotonic() - self.started > 0.3:
                self.finish("aborted", "no progress on [drivetrain/x_meters] for 0.3s")
            self.publish_state()
            self.inst.flush()
            time.sleep(LOOP)


@pytest.fixture(scope="module")
def robot(tmp_path_factory):
    port = _free_port()
    bot = FakeRobot(port, str(tmp_path_factory.mktemp("nt") / "nt.json"))
    bot.port = port
    yield bot
    bot.close()


@pytest.fixture(scope="module")
def client(robot):
    c = RobotClient(server="127.0.0.1", port=robot.port, default_timeout=10.0)
    assert c.wait_for_connection(5.0), "client never connected to the fake robot"
    manifest = c.get_manifest(timeout=5.0)
    assert [s.name for s in manifest.commands] == ["drive_distance", "idle", "self_abort"]
    # Let the baseline request ids reach the robot before any test sends a real one.
    time.sleep(0.2)
    yield c
    c.close()


@pytest.fixture(autouse=True)
def reset(robot):
    robot.drift = 0.0
    robot.short_by = 0.0
    robot.ignore_requests = False
    yield


def test_manifest_carries_monitoring_hints(client):
    spec = client.get_manifest().get("drive_distance")
    assert spec.tracked_state == [X, Y, H]
    assert spec.has_watchdog is True
    assert client.get_manifest().get("idle").tracked_state == []


def test_finished_run_has_trace_expected_and_no_discrepancies(client, robot):
    x0 = robot.x
    result = client.run_command("drive_distance", {"meters": 1.0})
    assert result.status == "finished", result
    assert result.ok
    trace = result.trace
    assert trace.expected[X] == pytest.approx(x0 + 1.0, abs=1e-3)
    assert trace.start_state[X] == pytest.approx(x0, abs=0.1)
    assert len(trace.samples) >= 3
    xs = [s.values[X] for s in trace.samples]
    assert xs == sorted(xs), "x should increase monotonically"
    assert trace.end_state[X] == pytest.approx(x0 + 1.0, abs=0.1)
    assert trace.discrepancies == []
    assert trace.violations == []
    assert client.get_trace("drive_distance") is trace


def test_short_run_reports_discrepancy(client, robot):
    robot.short_by = 0.5
    result = client.run_command("drive_distance", {"meters": 1.0})
    assert result.status == "finished"
    assert not result.ok
    assert len(result.trace.discrepancies) == 1
    assert result.trace.discrepancies[0].startswith(f"{X}: expected")


def test_client_rule_cancels_drifting_drive(client, robot):
    robot.drift = 1.0  # 1 m/s sideways: crosses the 0.2 m path tolerance in ~0.2 s
    result = client.run_command("drive_distance", {"meters": 3.0})
    assert result.status == "aborted", result
    assert result.message.startswith("client monitor:")
    assert "off the straight line" in result.message
    violation = result.trace.violations[-1]
    assert violation.source == "client"
    assert robot.active is None, "robot should have seen the cancel"
    assert client.get_status("drive_distance") == "interrupted"


def test_robot_abort_is_recorded_as_robot_violation(client, robot):
    result = client.run_command("self_abort")
    assert result.status == "aborted"
    assert result.message.startswith("no progress")
    assert [v.source for v in result.trace.violations] == ["robot"]


def test_check_in_returns_running_then_cancel(client, robot):
    result = client.run_command("idle", check_in=0.3)
    assert result.status == "running"
    assert 0.25 < result.elapsed_seconds < 1.0
    assert result.trace.running

    again = client.wait_for_command("idle", check_in=0.2)
    assert again.status == "running"
    assert again.trace is result.trace

    client.cancel_command("idle")
    final = client.wait_for_command("idle", check_in=2.0)
    assert final.status == "interrupted"
    assert final.trace.status == "interrupted"
    # A further wait just reports the stored outcome.
    assert client.wait_for_command("idle").status == "interrupted"
    assert client.wait_for_command("self_abort").status == "aborted"


def test_each_request_starts_exactly_one_run(client, robot):
    before = robot.runs_started
    for _ in range(3):
        assert client.run_command("drive_distance", {"meters": 0.2}).status == "finished"
    # The request id stays published between calls; the robot must not re-trigger on it.
    time.sleep(0.2)
    assert robot.runs_started == before + 3


def test_unacknowledged_request_is_reported(client, robot):
    robot.ignore_requests = True
    result = client.run_command("drive_distance", {"meters": 1.0})
    assert result.status == "unacknowledged", result
    assert "did not pick up" in result.message
    assert client.get_trace("drive_distance").status == "unacknowledged"
    # Nothing was left running on the robot.
    assert robot.active is None
    robot.ignore_requests = False
    # Robot state is consistent afterwards: the next request runs normally.
    assert client.run_command("drive_distance", {"meters": 0.2}).status == "finished"


def test_langchain_tools_round_trip(client, robot):
    from robot_llm.tools import build_tools

    tools = {t.name: t for t in build_tools(client)}
    assert {"drive_distance", "wait_for_command", "cancel_command", "get_command_trace"} <= tools.keys()
    assert "RUNNING" in tools["drive_distance"].description

    text = tools["drive_distance"].invoke({"meters": 0.5})
    assert text.startswith("FINISHED: drive_distance completed")
    assert "columns: t, drivetrain/x_meters" in text

    client.check_in_seconds = 0.3
    text = tools["idle"].invoke({})
    assert text.startswith("RUNNING: idle is still running")
    text = tools["cancel_command"].invoke({"name": "idle"})
    assert text.startswith("INTERRUPTED: idle did not complete")
    assert tools["cancel_command"].invoke({"name": "idle"}) == "idle is not running (status interrupted)."

    full = json.loads(tools["get_command_trace"].invoke({"name": "drive_distance"}))
    assert full["status"] == "finished" and full["sample_count"] >= 3
