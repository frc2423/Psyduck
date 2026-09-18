"""LangChain agent wiring."""

from __future__ import annotations

import os
from collections.abc import Iterator
from typing import Any

from langchain.agents import create_agent
from langchain.chat_models import init_chat_model
from langchain_core.messages import AIMessage, BaseMessage, HumanMessage, ToolMessage
from langchain_core.tools import BaseTool
from langgraph.checkpoint.memory import InMemorySaver

DEFAULT_MODEL = os.environ.get("OPENAI_MODEL", "gpt-5-mini")

SYSTEM_PROMPT = """\
You are the operator interface for an FRC (FIRST Robotics Competition) robot. You control the
robot exclusively by calling the tools you are given; each tool schedules a command on the robot
and returns when it finishes.

Guidelines:
- Robot conventions: distances are in meters, angles in degrees, and positive rotation is
  counter-clockwise (turning left). "Forward" means along the robot's current heading.
- Before multi-step manoeuvres, or when the user asks about the robot, call `get_robot_state`.
- Break compound requests into a sequence of tool calls and run them in order, one at a time.
  Do not call a movement tool until the previous one has returned.
- If a tool reports REJECTED, INTERRUPTED or TIMEOUT, stop the sequence and tell the user what
  happened rather than retrying blindly. A common cause is the robot being disabled.
- Stay within the parameter limits described in the tool schemas. If the user asks for something
  outside the robot's capabilities, say so instead of improvising.
- If the user says "stop", call `cancel_all_commands` immediately.
- Be concise. Report what the robot actually did, based on tool results, not what you intended.
"""


def create_robot_agent(tools: list[BaseTool], model: str | None = None):
    """Build a tool-calling agent with in-memory conversation history."""
    llm = init_chat_model(model or DEFAULT_MODEL, model_provider="openai")
    return create_agent(
        model=llm,
        tools=tools,
        system_prompt=SYSTEM_PROMPT,
        checkpointer=InMemorySaver(),
    )


def run_turn(agent: Any, text: str, thread_id: str = "operator") -> Iterator[BaseMessage]:
    """Send one user message and yield each new message (tool calls, tool results, replies)."""
    config = {"configurable": {"thread_id": thread_id}}
    for update in agent.stream(
        {"messages": [HumanMessage(content=text)]}, config=config, stream_mode="updates"
    ):
        for node_output in update.values():
            if not isinstance(node_output, dict):
                continue
            yield from node_output.get("messages", [])


def describe(message: BaseMessage) -> str | None:
    """Render a message for the console; returns None for ones not worth printing."""
    if isinstance(message, AIMessage):
        lines = [f"  -> {tc['name']}({_fmt_args(tc['args'])})" for tc in message.tool_calls]
        if message.text:
            lines.append(message.text)
        return "\n".join(lines) if lines else None
    if isinstance(message, ToolMessage):
        return f"  <- {message.content}"
    return None


def _fmt_args(args: dict[str, Any]) -> str:
    return ", ".join(f"{k}={v!r}" for k, v in args.items())
