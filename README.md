# Robot LLM command experiment

Controls a robot over NetworkTables using an LLM. The program is a Python script that uses
LangChain, ntcore, and OpenAI models. The robot is controlled through tools that trigger commands
(from the command-based framework) over NetworkTables.

```
 you> pick up a game piece, then drive 2 meters forward and score high
   -> get_robot_state()
   <- {"intake/has_game_piece": false, ...}
   -> intake_game_piece()
   <- OK: intake_game_piece finished in 1.9s.
   -> drive_distance(meters=2.0)
   <- OK: drive_distance finished in 3.1s.
   -> score(level='high')
   <- OK: score finished in 4.2s.
 Picked up a game piece, drove 2 m forward and scored on the high level.
```

## How it works

```
 ┌──────────────────────────┐   NetworkTables    ┌──────────────────────────────┐
 │ Python: robot_llm        │ ◄────────────────► │ Robot (Java, command-based)  │
 │  - reads /LLM/manifest   │                    │  LlmCommands registry        │
 │  - builds one LangChain  │  params + run ───► │   - publishes manifest       │
 │    tool per command      │  ◄── status/count  │   - schedules Command on run │
 │  - OpenAI agent loop     │  ◄── state/*       │   - reports completion       │
 └──────────────────────────┘                    └──────────────────────────────┘
```

1. Robot code registers commands with `LlmCommands.register(name)` — a name, description and typed
   parameters plus a factory that builds the `Command`.
2. The registry publishes a JSON **manifest** to `/LLM/manifest`. The Python client reads it and
   generates a LangChain tool (with a Pydantic schema: types, min/max, choices) for each command,
   so **adding a command in Java automatically adds a tool for the LLM**.
3. When the model calls a tool, the client writes the parameters to
   `/LLM/commands/<name>/params/*`, sets `run = true`, and waits for `completedCount` to increment.
   The robot schedules the command, then reports `status` (`finished` / `interrupted` /
   `rejected`) and `lastResult`, which is returned to the model.
4. Subsystems publish telemetry under `/LLM/state/*`; the model reads it via `get_robot_state`.

### NetworkTables protocol (`/LLM`)

| Key | Written by | Meaning |
| --- | --- | --- |
| `manifest` | robot | JSON list of commands and parameters |
| `commands/<name>/params/<p>` | client | parameter values for the next run |
| `commands/<name>/run` | client → robot resets | set `true` to schedule |
| `commands/<name>/cancel` | client → robot resets | set `true` to cancel a running instance |
| `commands/<name>/status` | robot | `idle`, `running`, `finished`, `interrupted`, `rejected` |
| `commands/<name>/completedCount` | robot | increments each time a run ends |
| `commands/<name>/lastResult` | robot | human-readable outcome |
| `cancelAll` | client → robot resets | cancel everything |
| `state/<key>` | robot | telemetry (`drivetrain/x_meters`, `arm/angle_degrees`, `robot/enabled`, ...) |

## Layout

```
src/robot_llm/
  nt_client.py   RobotClient: NT connection, manifest, run_command, get_state
  tools.py       manifest -> LangChain StructuredTools (+ get_robot_state, cancel_all_commands, wait_seconds)
  agent.py       create_agent() wiring, system prompt, console rendering
  cli.py         interactive REPL / one-shot runner  (entry point: `robot-llm`)
  server.py      FastAPI WebSocket bridge for the web frontend  (entry point: `robot-llm-server`)
robot-llm-frontend/
  src/           Vite + React + MUI + zustand web UI: chat, running command, robot state, command list
ExampleRobotProject/src/main/java/frc/robot/
  llm/LlmCommands.java         registry + NT protocol; call periodic() from robotPeriodic()
  llm/LlmCommandBuilder.java   fluent registration API
  llm/LlmParams.java           typed access to the parameters the client sent
  llm/ParamSpec.java           parameter description published in the manifest
  subsystems/                  simulated Drivetrain, Arm and Intake (no hardware needed)
  RobotContainer.java          example command registrations
ExampleRobotProject/src/test/java/frc/robot/llm/LlmCommandsTest.java   protocol tests under HAL sim
```

### Example commands exposed by the robot

| Tool | Parameters | Description |
| --- | --- | --- |
| `drive_distance` | `meters` (−10..10) | drive straight, signed distance |
| `turn_to_heading` | `degrees` (−180..180) | rotate to an absolute heading |
| `turn_by` | `degrees` (−360..360) | rotate relative to the current heading |
| `stop_driving` | – | stop the drivetrain |
| `arm_to_angle` | `degrees` (0..120) | move the arm to an angle |
| `arm_to_preset` | `preset` ∈ stowed, intake, low, high | move the arm to a named position |
| `intake_game_piece` | – | lower arm and run rollers until a piece is held |
| `eject_game_piece` | – | reverse rollers briefly |
| `score` | `level` ∈ low, high | arm up → eject → stow (no-op if nothing held) |
| `say` | `message` | print to the robot console |

### Adding a command

```java
LlmCommands.register("drive_distance")
    .description("Drive the robot in a straight line. Positive is forward, negative backward.")
    .doubleParam("meters", "Signed distance to travel in meters.", -10.0, 10.0)
    .timeout(20.0)
    .command(p -> m_drivetrain.driveDistance(p.getDouble("meters")));
```

The factory must return a **new** `Command` each call (the registry wraps every run in a
composition). Parameter types: `doubleParam`, `integerParam`, `booleanParam`, `stringParam`,
`choiceParam`. Publish telemetry with `LlmCommands.publishState("arm/angle_degrees", angle)`.
Descriptions are shown verbatim to the model, so be explicit about units and sign conventions.

## Installation

Uses [uv](https://docs.astral.sh/uv/) to install.

```sh
uv sync
cp .env.example .env       # then put your OPENAI_API_KEY in .env
```

## Running

1. Start the robot. For simulation, in `ExampleRobotProject`:
   ```sh
   ./gradlew simulateJava
   ```
   In the sim GUI, set the **Robot State** to *Teleoperated* — commands are rejected while the
   robot is disabled.
2. Start the agent (defaults to `localhost`, i.e. the simulator):
   ```sh
   uv run robot-llm                 # interactive
   uv run robot-llm "turn left 90 degrees"   # one-shot
   uv run robot-llm --team 2423     # a real robot
   uv run robot-llm --model gpt-5   # different OpenAI model (or set OPENAI_MODEL)
   ```
   REPL commands: `/commands`, `/state`, `/cancel`, `/quit`.

### Web frontend

The React app talks to a small FastAPI bridge that wraps the same `RobotClient` and agent over a
single WebSocket (`/ws`). Start the robot (or simulator) as above, then:

```sh
uv run robot-llm-server            # bridge on http://127.0.0.1:8000 (same --server/--team/--model flags)
cd robot-llm-frontend && npm install && npm run dev
```

Open the URL Vite prints (it proxies `/ws` and `/api` to the bridge). The page shows the chat with
inline tool calls/results, the currently running command with elapsed time and a cancel button,
live telemetry from `/LLM/state`, and every command from the manifest with its parameters and
status. **Stop all** in the toolbar sends `cancelAll`.

Bridge protocol (JSON over the WebSocket): server sends `hello`/`manifest` (command specs),
`snapshot` (robot connection, telemetry, per-command status — only when something changes),
`turn` (`start`/`end`), `message` (`assistant`, `tool_call`, `tool_result`) and `error`; the client
sends `chat {text}`, `cancel_all` and `cancel {name}`. See `src/robot_llm/server.py`.

## Tests

```sh
cd ExampleRobotProject && ./gradlew test
```

The Java tests run the robot code under HAL simulation and drive it through the NetworkTables
protocol (manifest contents, rejection while disabled, completion, cancellation, sequences).

## Links

- ntcore docs: https://robotpy.readthedocs.io/projects/robotpy/en/stable/api.html#ntcore-api
- langchain docs: https://docs.langchain.com/oss/python/langchain/overview
- wpilib docs: https://docs.wpilib.org/en/stable/index.html
