"""Client-side monitoring of a running robot command.

While ``RobotClient.run_command`` waits for a command to finish it records a :class:`CommandTrace`:
the tracked state keys sampled every ~100 ms, the expected end values the robot published, and any
rule violations. Rules are plain callables evaluated after every sample; the first one that
returns a reason cancels the command. When the run ends the final state is compared against the
expected values and any discrepancies are recorded, so the model can tell "finished where it
should" from "finished, but not as intended".

The robot runs its own checks at loop rate (see ``LlmCommandBuilder.watchdog``); these rules are
the second line of defence and the source of the evidence the model reasons over.
"""

from __future__ import annotations

import json
import math
from collections.abc import Callable
from dataclasses import asdict, dataclass, field
from typing import Any

# Distance (m) the robot may drift from the straight line between its start and expected pose.
PATH_TOLERANCE_METERS = 0.2
# Tolerances used when comparing final state to expected values, chosen by key suffix.
TOLERANCE_BY_SUFFIX: dict[str, float] = {"_meters": 0.1, "_degrees": 3.0, "_mps": 0.1}
DEFAULT_NUMERIC_TOLERANCE = 0.05
# Change below which a numeric value is considered unchanged by the stall rule.
STALL_EPSILON = 1e-3


@dataclass
class Sample:
    t: float  # seconds since the run started
    values: dict[str, Any]


@dataclass
class Violation:
    t: float
    source: str  # "client" (a rule below) or "robot" (its stall check / watchdog)
    message: str
    values: dict[str, Any] = field(default_factory=dict)


@dataclass
class CommandTrace:
    """Everything observed about one execution of a command."""

    name: str
    args: dict[str, Any]
    tracked: list[str]
    expected: dict[str, Any] = field(default_factory=dict)
    start_state: dict[str, Any] = field(default_factory=dict)
    samples: list[Sample] = field(default_factory=list)
    violations: list[Violation] = field(default_factory=list)
    status: str = "running"
    message: str = ""
    elapsed_seconds: float = 0.0
    end_state: dict[str, Any] | None = None
    discrepancies: list[str] = field(default_factory=list)

    @property
    def latest(self) -> Sample | None:
        return self.samples[-1] if self.samples else None

    @property
    def running(self) -> bool:
        return self.status == "running"

    def add_sample(self, t: float, state: dict[str, Any]) -> Sample:
        sample = Sample(round(t, 2), {k: state.get(k) for k in self.tracked})
        self.samples.append(sample)
        return sample

    def add_violation(self, t: float, source: str, message: str) -> Violation:
        latest = self.latest
        violation = Violation(round(t, 2), source, message, dict(latest.values) if latest else {})
        self.violations.append(violation)
        return violation

    def downsampled(self, max_points: int = 10) -> list[Sample]:
        """Evenly spaced subset of the samples, always including the first and last."""
        n = len(self.samples)
        if n <= max_points:
            return list(self.samples)
        step = (n - 1) / (max_points - 1)
        return [self.samples[round(i * step)] for i in range(max_points)]

    def to_dict(self, max_points: int | None = None) -> dict[str, Any]:
        data = asdict(self)
        if max_points is not None:
            data["samples"] = [asdict(s) for s in self.downsampled(max_points)]
            data["sample_count"] = len(self.samples)
        return data

    def compute_discrepancies(self) -> list[str]:
        """Compare ``end_state`` to ``expected``; records and returns the mismatches."""
        self.discrepancies = []
        if not self.expected or self.end_state is None:
            return self.discrepancies
        for key, want in self.expected.items():
            got = self.end_state.get(key)
            if not values_match(key, want, got):
                self.discrepancies.append(f"{key}: expected {_fmt(want)}, got {_fmt(got)}")
        return self.discrepancies


# ---------------------------------------------------------------------------------- rules

# A rule sees the trace so far and the full current robot state; it returns a reason to abort.
Rule = Callable[[CommandTrace, dict[str, Any]], str | None]


def robot_disabled(trace: CommandTrace, state: dict[str, Any]) -> str | None:
    if state.get("robot/enabled") is False:
        return "robot became disabled"
    return None


def entered_avoidance_zone(trace: CommandTrace, state: dict[str, Any]) -> str | None:
    x, y = state.get("drivetrain/x_meters"), state.get("drivetrain/y_meters")
    if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
        return None
    for zone in _zones(state):
        if zone["min_x"] <= x <= zone["max_x"] and zone["min_y"] <= y <= zone["max_y"]:
            return f"robot is inside avoidance zone '{zone['name']}' at ({x:.2f}, {y:.2f})"
    return None


def path_deviation(trace: CommandTrace, state: dict[str, Any]) -> str | None:
    """For commands that expect a target pose: abort if the robot leaves the straight line to it."""
    keys = ("drivetrain/x_meters", "drivetrain/y_meters")
    if not all(k in trace.expected and k in trace.start_state for k in keys):
        return None
    x, y = state.get(keys[0]), state.get(keys[1])
    if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
        return None
    ax, ay = trace.start_state[keys[0]], trace.start_state[keys[1]]
    bx, by = trace.expected[keys[0]], trace.expected[keys[1]]
    off = _distance_from_line(x, y, ax, ay, bx, by)
    if off > PATH_TOLERANCE_METERS:
        return (
            f"robot is {off:.2f} m off the straight line from ({ax:.2f}, {ay:.2f}) to "
            f"({bx:.2f}, {by:.2f}); now at ({x:.2f}, {y:.2f})"
        )
    return None


def stalled_for(seconds: float) -> Rule:
    """Rule: no tracked numeric value has changed in the last ``seconds`` of samples."""

    def rule(trace: CommandTrace, state: dict[str, Any]) -> str | None:
        latest = trace.latest
        if latest is None or latest.t < seconds:
            return None
        window = [s for s in trace.samples if s.t >= latest.t - seconds]
        if len(window) < 2:
            return None
        for key in trace.tracked:
            first = window[0].values.get(key)
            for s in window[1:]:
                if _changed(first, s.values.get(key)):
                    return None
        return f"no change in {trace.tracked} for {seconds:g}s (values {latest.values})"

    return rule


DEFAULT_RULES: list[Rule] = [robot_disabled, entered_avoidance_zone, path_deviation]


# ------------------------------------------------------------------------------ formatting


def format_result(trace: CommandTrace, *, max_points: int = 8) -> str:
    """Render a trace as the text a tool returns to the model."""
    name, status = trace.name, trace.status
    head = f"{status.upper()}: {name}"
    lines: list[str] = []

    if status == "running":
        lines.append(f"{head} is still running after {trace.elapsed_seconds:.1f}s (check-in).")
    elif status == "finished":
        if trace.discrepancies:
            lines.append(
                f"FINISHED_WITH_DISCREPANCIES: {name} reported completion after "
                f"{trace.elapsed_seconds:.1f}s, but the end state does not match what was expected."
            )
        else:
            lines.append(f"{head} completed in {trace.elapsed_seconds:.1f}s.")
    else:
        lines.append(f"{head} did not complete ({trace.elapsed_seconds:.1f}s): {trace.message}")

    for v in trace.violations:
        lines.append(f"Violation at t={v.t:.1f}s ({v.source}): {v.message}")
    for d in trace.discrepancies:
        lines.append(f"Discrepancy: {d}")

    if trace.expected:
        current = trace.end_state if trace.end_state is not None else (trace.latest.values if trace.latest else {})
        parts = [f"{k}: expected {_fmt(v)}, now {_fmt(current.get(k))}" for k, v in trace.expected.items()]
        lines.append("Expected vs actual: " + "; ".join(parts))

    if trace.samples:
        points = trace.downsampled(max_points)
        lines.append(
            f"Trace ({len(points)} of {len(trace.samples)} samples; columns: t, "
            + ", ".join(trace.tracked)
            + "):"
        )
        for s in points:
            lines.append(f"  {s.t:5.1f}s | " + " | ".join(_fmt(s.values.get(k)) for k in trace.tracked))

    if status == "running":
        lines.append(
            f"Decide: call wait_for_command('{name}') to keep waiting, or "
            f"cancel_command('{name}') if the trace shows it is not doing what you intended."
        )
    return "\n".join(lines)


# --------------------------------------------------------------------------------- helpers


def values_match(key: str, want: Any, got: Any) -> bool:
    if isinstance(want, bool) or isinstance(got, bool):
        return want == got
    if isinstance(want, (int, float)) and isinstance(got, (int, float)):
        tol = next((t for suffix, t in TOLERANCE_BY_SUFFIX.items() if key.endswith(suffix)), None)
        if tol is None:
            tol = DEFAULT_NUMERIC_TOLERANCE
        diff = got - want
        if key.endswith("heading_degrees"):
            diff = (diff + 180.0) % 360.0 - 180.0
        return abs(diff) <= tol
    return want == got


def _changed(before: Any, after: Any) -> bool:
    if isinstance(before, (int, float)) and isinstance(after, (int, float)) and not isinstance(before, bool):
        return abs(after - before) > STALL_EPSILON
    return before != after


def _zones(state: dict[str, Any]) -> list[dict[str, Any]]:
    raw = state.get("avoidance_zones")
    if not isinstance(raw, str) or not raw:
        return []
    try:
        zones = json.loads(raw)
    except ValueError:
        return []
    return [z for z in zones if isinstance(z, dict) and {"min_x", "min_y", "max_x", "max_y"} <= z.keys()]


def _distance_from_line(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    dx, dy = bx - ax, by - ay
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return math.hypot(px - ax, py - ay)
    return abs(dy * (px - ax) - dx * (py - ay)) / length


def _fmt(value: Any) -> str:
    if value is None:
        return "?"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return f"{value:.2f}"
    return str(value)
