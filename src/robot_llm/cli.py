"""Interactive console: type natural-language instructions, the agent drives the robot."""

from __future__ import annotations

import argparse
import json
import os
import sys

from dotenv import load_dotenv

from robot_llm.agent import DEFAULT_MODEL, create_robot_agent, describe, run_turn
from robot_llm.nt_client import RobotClient
from robot_llm.tools import build_tools

HELP = """\
Type an instruction for the robot, or one of:
  /commands   list the commands the robot exposes
  /state      dump robot telemetry
  /cancel     cancel everything running on the robot
  /quit       exit
"""


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Drive an FRC robot with an LLM.")
    target = parser.add_mutually_exclusive_group()
    target.add_argument(
        "--server", default=None, help="NetworkTables server host (default: localhost, for sim)"
    )
    target.add_argument("--team", type=int, default=None, help="FRC team number (real robot)")
    parser.add_argument("--model", default=DEFAULT_MODEL, help=f"OpenAI model (default: {DEFAULT_MODEL})")
    parser.add_argument(
        "--timeout", type=float, default=30.0, help="Max seconds to wait for a command (default 30)"
    )
    parser.add_argument(
        "--connect-timeout", type=float, default=10.0, help="Seconds to wait for the robot"
    )
    parser.add_argument("prompt", nargs="*", help="Run a single instruction and exit")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    load_dotenv()
    args = parse_args(argv)

    if not os.environ.get("OPENAI_API_KEY"):
        print("OPENAI_API_KEY is not set (put it in .env or the environment).", file=sys.stderr)
        return 2

    where = f"team {args.team}" if args.team else (args.server or "localhost")
    print(f"Connecting to robot at {where} ...", flush=True)
    client = RobotClient(server=args.server, team=args.team, default_timeout=args.timeout)
    if not client.wait_for_connection(args.connect_timeout):
        print("Could not connect to the robot's NetworkTables server.", file=sys.stderr)
        print("For simulation: run `./gradlew simulateJava` in ExampleRobotProject.", file=sys.stderr)
        return 1

    try:
        tools = build_tools(client)
    except TimeoutError as e:
        print(e, file=sys.stderr)
        return 1

    manifest = client.get_manifest()
    print(f"Connected. Robot exposes {len(manifest.commands)} commands; using model {args.model}.")
    agent = create_robot_agent(tools, model=args.model)

    if args.prompt:
        return _turn(agent, " ".join(args.prompt))

    print(HELP)
    while True:
        try:
            line = input("you> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        if line in ("/quit", "/exit", "/q"):
            break
        if line == "/help":
            print(HELP)
        elif line == "/commands":
            for cmd in manifest.commands:
                params = ", ".join(f"{p.name}: {p.type}" for p in cmd.parameters)
                print(f"  {cmd.name}({params})\n      {cmd.description}")
        elif line == "/state":
            print(json.dumps(client.get_state(), indent=1))
        elif line == "/cancel":
            client.cancel_all()
            print("cancel sent")
        else:
            _turn(agent, line)

    client.close()
    return 0


def _turn(agent, text: str) -> int:
    try:
        for message in run_turn(agent, text):
            rendered = describe(message)
            if rendered:
                print(rendered, flush=True)
    except KeyboardInterrupt:
        print("\n(interrupted)")
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
