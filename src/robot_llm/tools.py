"""Build LangChain tools from the robot's published command manifest."""

from __future__ import annotations

import json
import time
from typing import Any, Literal

from langchain_core.tools import BaseTool, StructuredTool, tool
from pydantic import BaseModel, Field, create_model

from robot_llm.monitor import format_result
from robot_llm.nt_client import CommandResult, CommandSpec, ParamSpec, RobotClient

_PY_TYPES: dict[str, type] = {
    "double": float,
    "integer": int,
    "boolean": bool,
    "string": str,
}


def _field_for(param: ParamSpec) -> tuple[Any, Any]:
    """Return a ``(annotation, Field)`` pair for ``pydantic.create_model``."""
    annotation: Any = _PY_TYPES.get(param.type, str)
    kwargs: dict[str, Any] = {"description": param.description}

    if param.choices:
        annotation = Literal[tuple(param.choices)]  # type: ignore[valid-type]
    if param.type in ("double", "integer"):
        if param.min is not None:
            kwargs["ge"] = param.min
        if param.max is not None:
            kwargs["le"] = param.max

    return annotation, Field(..., **kwargs)


def _args_schema_for(spec: CommandSpec) -> type[BaseModel]:
    fields = {p.name: _field_for(p) for p in spec.parameters}
    return create_model(f"{_pascal(spec.name)}Args", **fields)  # type: ignore[call-overload]


def _pascal(snake: str) -> str:
    return "".join(part.capitalize() for part in snake.split("_"))


def describe_result(result: CommandResult) -> str:
    """Text returned to the model for a command run (or a check-in on one)."""
    if result.trace is not None:
        return format_result(result.trace)
    if result.ok:
        return f"FINISHED: {result.name} completed in {result.elapsed_seconds:.1f}s."
    return f"{result.status.upper()}: {result.name} did not complete. {result.message}"


def command_tool(client: RobotClient, spec: CommandSpec) -> BaseTool:
    """Create a tool that triggers ``spec`` on the robot and waits for it to finish.

    The tool blocks until the command ends or the check-in budget elapses, then returns the
    outcome together with a trace of the tracked telemetry, any rule violations, and how the
    final state compares to what the command was expected to achieve.
    """

    def _run(**kwargs: Any) -> str:
        return describe_result(client.run_command(spec.name, kwargs))

    description = spec.description or f"Run the robot command '{spec.name}'."
    if spec.timeout_seconds > 0:
        description += f" Times out after {spec.timeout_seconds:g}s."
    if spec.tracked_state:
        description += (
            " Returns a trace of " + ", ".join(spec.tracked_state) + " recorded while it ran."
        )
    check_in = spec.check_in_seconds or client.check_in_seconds
    if spec.timeout_seconds == 0 or spec.timeout_seconds > check_in:
        description += (
            f" If still running after {check_in:g}s it returns RUNNING with the trace so far;"
            " use wait_for_command to continue."
        )

    return StructuredTool.from_function(
        func=_run,
        name=spec.name,
        description=description,
        args_schema=_args_schema_for(spec),
    )


def builtin_tools(client: RobotClient) -> list[BaseTool]:
    """Tools that exist regardless of what the robot exposes."""

    @tool
    def get_robot_state() -> str:
        """Read the robot's current telemetry (pose, arm angle, whether it holds a game piece,
        enabled state, ...). Call this before planning multi-step actions or when asked about
        the robot's status."""
        state = client.get_state()
        if not state:
            return "No state published yet. Is the robot connected?"
        return json.dumps(state, indent=1)

    @tool
    def cancel_all_commands() -> str:
        """Immediately cancel every command running on the robot. Use for 'stop' or emergencies."""
        client.cancel_all()
        return "Cancel request sent."

    @tool
    def wait_for_command(name: str) -> str:
        """Keep waiting on a command tool that returned RUNNING. Blocks for another check-in
        period and returns the updated trace, or the final outcome once the command ends."""
        return describe_result(client.wait_for_command(name))

    @tool
    def cancel_command(name: str) -> str:
        """Cancel one running command by name (e.g. after a RUNNING check-in shows it is not
        doing what you intended). Returns the final trace."""
        result = client.wait_for_command(name, check_in=0.0)
        if result.status != "running":
            if client.get_status(name) == "running":
                # Running on the robot but not tracked here (started by another client, or a
                # run this client lost track of): cancel it anyway rather than leave it moving.
                client.cancel_command(name)
                return f"{name} was running outside this session; cancel request sent."
            return f"{name} is not running (status {result.status})."
        client.cancel_command(name)
        return describe_result(client.wait_for_command(name, check_in=2.0))

    @tool
    def get_command_trace(name: str) -> str:
        """Full recorded trace of the current or most recent run of a command: expected values,
        every sample of the tracked telemetry, violations and discrepancies. Use it to diagnose
        a run whose summary looked wrong."""
        trace = client.get_trace(name)
        if trace is None:
            return f"No trace recorded for {name!r} yet."
        return json.dumps(trace.to_dict(max_points=60), indent=1)

    @tool
    def wait_seconds(seconds: float) -> str:
        """Pause for a number of seconds (max 10) before continuing, e.g. to let motion settle."""
        seconds = max(0.0, min(float(seconds), 10.0))
        time.sleep(seconds)
        return f"Waited {seconds:g}s."

    return [
        get_robot_state,
        cancel_all_commands,
        wait_for_command,
        cancel_command,
        get_command_trace,
        wait_seconds,
    ]


def build_tools(client: RobotClient) -> list[BaseTool]:
    """All tools for the agent: one per robot command, plus the built-ins."""
    manifest = client.get_manifest()
    return [command_tool(client, spec) for spec in manifest.commands] + builtin_tools(client)
