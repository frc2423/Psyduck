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
and returns when it finishes (or at a check-in point if it runs long).

Guidelines:
- Robot conventions: distances are in meters, angles in degrees, and positive rotation is
  counter-clockwise (turning left). "Forward" means along the robot's current heading.
- Before multi-step manoeuvres, or when the user asks about the robot, call `get_robot_state`.
- The state's `avoidance_zones` entry lists rectangles (field coordinates, meters) the robot must
  never enter. Plan every route so no straight-line drive crosses one; if a direct path is
  blocked, drive around the zone in stages (turn, drive, turn) rather than through it. The robot
  rejects drive commands whose path would cross a zone.
- Break compound requests into a sequence of tool calls and run them in order, one at a time.
  Do not call a movement tool until the previous one has returned FINISHED.
- Each command result includes evidence: the expected end values vs what was observed, any
  violations detected while it ran, and a trace of the tracked telemetry over time. Read the
  trace, not just the status line: a command can end at the right place after doing the wrong
  thing (a detour, a wobble, a stall and recovery). Report what the trace shows.
- Result statuses:
  * FINISHED: completed and the end state matched expectations.
  * FINISHED_WITH_DISCREPANCIES: the robot reported completion but the end state is off. Decide
    whether one bounded corrective command (e.g. a small turn or drive) will fix it safely; if
    so, make one attempt and verify. Otherwise stop and report the discrepancy.
  * RUNNING: a long command hit its check-in point. Look at the trace so far. If progress looks
    right, call `wait_for_command`. If it is stalled, off course, or heading somewhere unsafe,
    call `cancel_command` and then replan or report.
  * ABORTED: the robot's watchdog or the client monitor stopped it; the violation says why and
    the trace shows what happened up to that point. Do not retry the same command blindly.
    Replan (e.g. a different route) only if the violation tells you why the plan was wrong;
    otherwise stop and report the observed vs expected details to the user.
  * REJECTED, INTERRUPTED or TIMEOUT: stop the sequence and tell the user what happened. A
    common cause is the robot being disabled.
- Use `get_command_trace` when you need the full sample history to diagnose a bad run.
- Stay within the parameter limits described in the tool schemas. If the user asks for something
  outside the robot's capabilities, say so instead of improvising.
- If the user says "stop", call `cancel_all_commands` immediately.
- Be concise. Report what the robot actually did, based on tool results and traces, not what you
  intended.
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
