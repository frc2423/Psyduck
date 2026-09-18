"""Build LangChain tools from the robot's published command manifest."""

from __future__ import annotations

import json
import time
from typing import Any, Literal

from langchain_core.tools import BaseTool, StructuredTool, tool
from pydantic import BaseModel, Field, create_model

from robot_llm.nt_client import CommandSpec, ParamSpec, RobotClient

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


def command_tool(client: RobotClient, spec: CommandSpec) -> BaseTool:
    """Create a tool that triggers ``spec`` on the robot and waits for it to finish."""

    def _run(**kwargs: Any) -> str:
        result = client.run_command(spec.name, kwargs)
        if result.ok:
            return f"OK: {spec.name} finished in {result.elapsed_seconds:.1f}s."
        return f"{result.status.upper()}: {spec.name} did not complete. {result.message}"

    description = spec.description or f"Run the robot command '{spec.name}'."
    if spec.timeout_seconds > 0:
        description += f" Times out after {spec.timeout_seconds:g}s."

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
    def wait_seconds(seconds: float) -> str:
        """Pause for a number of seconds (max 10) before continuing, e.g. to let motion settle."""
        seconds = max(0.0, min(float(seconds), 10.0))
        time.sleep(seconds)
        return f"Waited {seconds:g}s."

    return [get_robot_state, cancel_all_commands, wait_seconds]


def build_tools(client: RobotClient) -> list[BaseTool]:
    """All tools for the agent: one per robot command, plus the built-ins."""
    manifest = client.get_manifest()
    return [command_tool(client, spec) for spec in manifest.commands] + builtin_tools(client)
