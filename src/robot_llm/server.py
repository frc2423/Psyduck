"""FastAPI bridge between the browser frontend and the robot agent.

Everything goes over one WebSocket at ``/ws``. Messages are JSON objects with a ``type``.

Server -> client
    ``hello``     ``{commands, model}``                 sent on connect (commands may be empty)
    ``manifest``  ``{commands}``                        sent when the robot's manifest appears
    ``snapshot``  ``{robotConnected, agentReady, state, commands}``  ~10 Hz, only on change
    ``turn``      ``{phase: "start" | "end"}``          brackets one agent turn
    ``message``   ``{role: assistant|tool_call|tool_result, ...}``    streamed during a turn
    ``error``     ``{message}``

Client -> server
    ``chat``        ``{text}``      send an instruction to the agent
    ``cancel_all``                  cancel everything on the robot
    ``cancel``      ``{name}``      cancel one command
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import os
import threading
import time
from contextlib import asynccontextmanager
from dataclasses import asdict
from typing import Any

import uvicorn
from dotenv import load_dotenv
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from langchain_core.messages import AIMessage, BaseMessage, ToolMessage

from robot_llm.agent import DEFAULT_MODEL, create_robot_agent, run_turn
from robot_llm.nt_client import CommandSpec, RobotClient
from robot_llm.tools import build_tools

log = logging.getLogger("robot_llm.server")

SNAPSHOT_INTERVAL = 0.1


def _command_json(spec: CommandSpec) -> dict[str, Any]:
    return {
        "name": spec.name,
        "description": spec.description,
        "timeoutSeconds": spec.timeout_seconds,
        "checkInSeconds": spec.check_in_seconds,
        "trackedState": spec.tracked_state,
        "parameters": [asdict(p) for p in spec.parameters],
    }


def _message_json(message: BaseMessage) -> list[dict[str, Any]]:
    """Flatten a LangChain message into the wire events the UI renders."""
    if isinstance(message, AIMessage):
        out: list[dict[str, Any]] = [
            {"type": "message", "role": "tool_call", "id": tc["id"], "name": tc["name"], "args": tc["args"]}
            for tc in message.tool_calls
        ]
        if message.text:
            out.append({"type": "message", "role": "assistant", "text": message.text})
        return out
    if isinstance(message, ToolMessage):
        return [
            {
                "type": "message",
                "role": "tool_result",
                "toolCallId": message.tool_call_id,
                "name": message.name,
                "content": message.text,
                "status": message.status,
            }
        ]
    return []


class Bridge:
    """Owns the robot client, the agent, and the set of connected browsers."""

    def __init__(self, client: RobotClient, model: str) -> None:
        self.client = client
        self.model = model
        self.agent: Any = None
        self.commands: list[CommandSpec] = []
        self.sockets: set[WebSocket] = set()
        self._turn_lock = asyncio.Lock()
        self._running_since: dict[str, float] = {}
        self._last_snapshot: str | None = None

    # ------------------------------------------------------------------ lifecycle

    async def run(self) -> None:
        """Background loop: acquire the manifest, then publish snapshots."""
        while True:
            if self.agent is None and self.client.connected:
                await self._try_build_agent()
            await self._publish_snapshot()
            await asyncio.sleep(SNAPSHOT_INTERVAL)

    async def _try_build_agent(self) -> None:
        try:
            manifest = await asyncio.to_thread(self.client.get_manifest, 0.5, True)
        except TimeoutError:
            return
        tools = build_tools(self.client)
        self.agent = create_robot_agent(tools, model=self.model)
        self.commands = manifest.commands
        log.info("Robot exposes %d commands; agent ready (%s)", len(self.commands), self.model)
        await self.broadcast({"type": "manifest", "commands": [_command_json(c) for c in self.commands]})

    # ------------------------------------------------------------------ snapshots

    def _snapshot(self) -> dict[str, Any]:
        now = time.time()
        commands: dict[str, Any] = {}
        for st in self.client.get_all_statuses():
            if st.status == "running":
                self._running_since.setdefault(st.name, now)
            else:
                self._running_since.pop(st.name, None)
            commands[st.name] = {
                "status": st.status,
                "lastResult": st.last_result,
                "completedCount": st.completed_count,
                "runningSince": self._running_since.get(st.name),
            }
        return {
            "type": "snapshot",
            "robotConnected": self.client.connected,
            "agentReady": self.agent is not None,
            "state": self.client.get_state() if self.client.connected else {},
            "commands": commands,
        }

    async def _publish_snapshot(self) -> None:
        if not self.sockets:
            return
        snapshot = self._snapshot()
        encoded = json.dumps(snapshot)
        if encoded == self._last_snapshot:
            return
        self._last_snapshot = encoded
        await self.broadcast(snapshot)

    async def broadcast(self, payload: dict[str, Any]) -> None:
        dead = []
        for ws in list(self.sockets):
            try:
                await ws.send_json(payload)
            except Exception:  # noqa: BLE001 - any send failure means the socket is gone
                dead.append(ws)
        for ws in dead:
            self.sockets.discard(ws)

    # ------------------------------------------------------------------ sockets

    async def handle(self, ws: WebSocket) -> None:
        await ws.accept()
        self.sockets.add(ws)
        try:
            await ws.send_json(
                {"type": "hello", "model": self.model, "commands": [_command_json(c) for c in self.commands]}
            )
            await ws.send_json(self._snapshot())
            while True:
                data = await ws.receive_json()
                await self._dispatch(ws, data)
        except WebSocketDisconnect:
            pass
        finally:
            self.sockets.discard(ws)

    async def _dispatch(self, ws: WebSocket, data: dict[str, Any]) -> None:
        kind = data.get("type")
        if kind == "chat":
            text = str(data.get("text", "")).strip()
            if text:
                await self._chat(text)
        elif kind == "cancel_all":
            self.client.cancel_all()
        elif kind == "cancel":
            name = data.get("name")
            if isinstance(name, str) and any(c.name == name for c in self.commands):
                self.client.cancel_command(name)
        else:
            await ws.send_json({"type": "error", "message": f"Unknown message type {kind!r}"})

    # ------------------------------------------------------------------ chat

    async def _chat(self, text: str) -> None:
        if self.agent is None:
            await self.broadcast({"type": "error", "message": "Robot is not connected; no commands available yet."})
            return
        if self._turn_lock.locked():
            await self.broadcast({"type": "error", "message": "The agent is still working on the previous request."})
            return

        async with self._turn_lock:
            await self.broadcast({"type": "turn", "phase": "start"})
            try:
                async for message in self._stream_turn(text):
                    for event in _message_json(message):
                        await self.broadcast(event)
            except Exception as e:  # noqa: BLE001 - surface anything the agent raises to the UI
                log.exception("agent turn failed")
                await self.broadcast({"type": "error", "message": f"{type(e).__name__}: {e}"})
            finally:
                await self.broadcast({"type": "turn", "phase": "end"})

    async def _stream_turn(self, text: str):
        """Run the (blocking) agent turn in a thread, yielding messages as they arrive."""
        loop = asyncio.get_running_loop()
        queue: asyncio.Queue[tuple[str, Any]] = asyncio.Queue()

        def worker() -> None:
            try:
                for message in run_turn(self.agent, text):
                    loop.call_soon_threadsafe(queue.put_nowait, ("message", message))
            except Exception as e:  # noqa: BLE001
                loop.call_soon_threadsafe(queue.put_nowait, ("error", e))
            finally:
                loop.call_soon_threadsafe(queue.put_nowait, ("done", None))

        threading.Thread(target=worker, name="agent-turn", daemon=True).start()
        while True:
            kind, payload = await queue.get()
            if kind == "message":
                yield payload
            elif kind == "error":
                raise payload
            else:
                return


# ---------------------------------------------------------------------- app factory


def create_app(
    *, server: str | None = None, team: int | None = None, model: str = DEFAULT_MODEL, timeout: float = 30.0
) -> FastAPI:
    client = RobotClient(server=server, team=team, default_timeout=timeout)
    bridge = Bridge(client, model)

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        task = asyncio.create_task(bridge.run())
        try:
            yield
        finally:
            task.cancel()
            client.close()

    app = FastAPI(title="robot-llm bridge", lifespan=lifespan)

    @app.get("/api/health")
    async def health() -> dict[str, Any]:
        return {"robotConnected": client.connected, "agentReady": bridge.agent is not None}

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket) -> None:
        await bridge.handle(ws)

    return app


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="WebSocket bridge for the robot LLM frontend.")
    target = parser.add_mutually_exclusive_group()
    target.add_argument("--server", default=None, help="NetworkTables server host (default: localhost)")
    target.add_argument("--team", type=int, default=None, help="FRC team number (real robot)")
    parser.add_argument("--model", default=DEFAULT_MODEL, help=f"OpenAI model (default: {DEFAULT_MODEL})")
    parser.add_argument("--timeout", type=float, default=30.0, help="Max seconds to wait for a command")
    parser.add_argument("--host", default="127.0.0.1", help="Bind address (default 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8000, help="Port (default 8000)")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    load_dotenv()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    args = parse_args(argv)
    if not os.environ.get("OPENAI_API_KEY"):
        log.warning("OPENAI_API_KEY is not set; chat turns will fail until it is.")
    app = create_app(server=args.server, team=args.team, model=args.model, timeout=args.timeout)
    uvicorn.run(app, host=args.host, port=args.port, log_level="info")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
