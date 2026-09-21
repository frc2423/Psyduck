"""Unit tests for the client-side trace, rules and result formatting (no NetworkTables)."""

from __future__ import annotations

import json

from robot_llm.monitor import (
    CommandTrace,
    entered_avoidance_zone,
    format_result,
    path_deviation,
    robot_disabled,
    stalled_for,
    values_match,
)

X, Y, H = "drivetrain/x_meters", "drivetrain/y_meters", "drivetrain/heading_degrees"


def drive_trace(meters: float = 2.0) -> CommandTrace:
    return CommandTrace(
        name="drive_distance",
        args={"meters": meters},
        tracked=[X, Y, H],
        expected={X: meters, Y: 0.0, H: 0.0},
        start_state={X: 0.0, Y: 0.0, H: 0.0},
    )


def test_downsampled_keeps_endpoints_and_bounds_size():
    trace = drive_trace()
    for i in range(50):
        trace.add_sample(i * 0.1, {X: i * 0.04})
    points = trace.downsampled(8)
    assert len(points) == 8
    assert points[0] is trace.samples[0]
    assert points[-1] is trace.samples[-1]
    assert [p.t for p in points] == sorted(p.t for p in points)
    # Small traces are returned whole.
    assert len(drive_trace().downsampled(8)) == 0


def test_values_match_uses_suffix_tolerances_and_wraps_heading():
    assert values_match(X, 2.0, 1.95)
    assert not values_match(X, 2.0, 1.8)
    assert values_match(H, 180.0, -179.0)
    assert not values_match(H, 90.0, 80.0)
    assert values_match("intake/has_game_piece", True, True)
    assert not values_match("intake/has_game_piece", True, False)


def test_compute_discrepancies_reports_mismatches():
    trace = drive_trace()
    trace.end_state = {X: 1.5, Y: 0.02, H: 0.4}
    assert trace.compute_discrepancies() == [f"{X}: expected 2.00, got 1.50"]
    trace.end_state = {X: 1.98, Y: 0.0, H: -1.0}
    assert trace.compute_discrepancies() == []


def test_path_deviation_rule():
    trace = drive_trace()
    assert path_deviation(trace, {X: 1.0, Y: 0.05}) is None
    reason = path_deviation(trace, {X: 1.0, Y: 0.5})
    assert reason is not None and "0.50 m off" in reason
    # Not applicable without an expected pose.
    assert path_deviation(CommandTrace("turn", {}, [H]), {X: 1.0, Y: 5.0}) is None


def test_avoidance_zone_and_disabled_rules():
    zones = json.dumps([{"name": "pit", "min_x": 1.0, "min_y": -1.0, "max_x": 2.0, "max_y": 1.0}])
    trace = drive_trace()
    assert entered_avoidance_zone(trace, {X: 0.5, Y: 0.0, "avoidance_zones": zones}) is None
    reason = entered_avoidance_zone(trace, {X: 1.5, Y: 0.0, "avoidance_zones": zones})
    assert reason is not None and "'pit'" in reason
    assert entered_avoidance_zone(trace, {X: 1.5, Y: 0.0, "avoidance_zones": "not json"}) is None
    assert robot_disabled(trace, {"robot/enabled": True}) is None
    assert robot_disabled(trace, {"robot/enabled": False}) == "robot became disabled"


def test_stalled_rule_needs_a_full_window_without_change():
    rule = stalled_for(1.0)
    trace = drive_trace()
    trace.add_sample(0.0, {X: 0.0, Y: 0.0, H: 0.0})
    trace.add_sample(0.5, {X: 0.0, Y: 0.0, H: 0.0})
    assert rule(trace, {}) is None  # window not long enough yet
    trace.add_sample(1.1, {X: 0.0, Y: 0.0, H: 0.0})
    assert rule(trace, {}) is not None
    trace.add_sample(1.2, {X: 0.3, Y: 0.0, H: 0.0})
    assert rule(trace, {}) is None


def test_format_result_running_finished_and_aborted():
    trace = drive_trace()
    trace.add_sample(0.0, {X: 0.0, Y: 0.0, H: 0.0})
    trace.add_sample(1.0, {X: 1.0, Y: 0.0, H: 0.0})
    trace.status, trace.elapsed_seconds = "running", 1.0
    text = format_result(trace)
    assert text.startswith("RUNNING: drive_distance is still running after 1.0s")
    assert "wait_for_command('drive_distance')" in text
    assert "columns: t, drivetrain/x_meters" in text
    assert "1.0s | 1.00 | 0.00 | 0.00" in text

    trace.status, trace.end_state = "finished", {X: 2.0, Y: 0.0, H: 0.0}
    trace.compute_discrepancies()
    assert format_result(trace).startswith("FINISHED: drive_distance completed")

    trace.end_state = {X: 1.2, Y: 0.0, H: 0.0}
    trace.compute_discrepancies()
    text = format_result(trace)
    assert text.startswith("FINISHED_WITH_DISCREPANCIES")
    assert "Discrepancy: drivetrain/x_meters: expected 2.00, got 1.20" in text

    trace.status, trace.message = "aborted", "client monitor: off course"
    trace.add_violation(0.9, "client", "off course")
    text = format_result(trace)
    assert text.startswith("ABORTED: drive_distance did not complete")
    assert "Violation at t=0.9s (client): off course" in text
