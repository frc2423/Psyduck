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
   <- FINISHED: drive_distance completed in 3.1s.
      Expected vs actual: drivetrain/x_meters: expected 2.00, now 1.99; ...
      Trace (8 of 31 samples; columns: t, drivetrain/x_meters, drivetrain/y_meters, ...):
          0.0s | 0.00 | 0.00 | ...
          ...
   -> score(level='high')
   <- OK: score finished in 4.2s.
 Picked up a game piece, drove 2 m forward and scored on the high level.
```

## How it works

```
 ┌──────────────────────────┐   NetworkTables    ┌──────────────────────────────┐
 │ Python: robot_llm        │ ◄────────────────► │ Robot (Java, command-based)  │
 │  - reads /LLM/manifest   │                    │  LlmCommands registry        │
 │  - builds one LangChain  │ params+request id► │   - publishes manifest       │
 │    tool per command      │  ◄── status/count  │   - schedules Command per id │
 │  - OpenAI agent loop     │  ◄── state/*       │   - reports completion       │
 └──────────────────────────┘                    └──────────────────────────────┘
```

1. Robot code registers commands with `LlmCommands.register(name)` — a name, description and typed
   parameters plus a factory that builds the `Command`.
2. The registry publishes a JSON **manifest** to `/LLM/manifest`. The Python client reads it and
   generates a LangChain tool (with a Pydantic schema: types, min/max, choices) for each command,
   so **adding a command in Java automatically adds a tool for the LLM**.
3. When the model calls a tool, the client writes the parameters to
   `/LLM/commands/<name>/params/*`, writes a fresh request id to `runRequest`, and waits for
   `completedCount` to increment. The robot acts exactly once per new id (echoing it in
   `runAck`), schedules the command, then reports `status` (`finished` / `interrupted` /
   `rejected`) and `lastResult`, which is returned to the model.
4. Subsystems publish telemetry under `/LLM/state/*`; the model reads it via `get_robot_state`.
5. **While a command runs it is monitored on both sides.** The robot evaluates the command's
   optional stall check and `watchdog` every loop and aborts the run itself if they fire. The
   client samples the command's `trackedState` keys every 100 ms into a *trace*, evaluates its
   own rules (robot disabled, inside an avoidance zone, off the straight line to the expected
   pose) and cancels on a violation. When the run ends the final state is compared with the
   `expected` values the robot published. The tool result the model sees contains the status,
   any violations, discrepancies, expected-vs-actual, and a downsampled trace — so it can tell
   "ended in the right place" from "ended in the right place the wrong way".
6. **Long commands check in.** A command tool blocks for at most a check-in budget (5 s by
   default, `checkIn(...)` per command). If the command is still running it returns `RUNNING`
   with the trace so far; the model then calls `wait_for_command` to keep going or
   `cancel_command` to stop it and replan.

### NetworkTables protocol (`/LLM`)

| Key | Written by | Meaning |
| --- | --- | --- |
| `manifest` | robot | JSON list of commands and parameters |
| `commands/<name>/params/<p>` | client | parameter values for the next run |
| `commands/<name>/runRequest` | client | request id; the robot runs once per id larger than the last it saw and adopts any other value as its baseline |
| `commands/<name>/runAck` | robot | last request id acted on (the client reports `unacknowledged` if its id is not echoed within 1 s) |
| `commands/<name>/cancelRequest` | client | request id; cancels the running instance |
| `commands/<name>/status` | robot | `idle`, `running`, `finished`, `interrupted`, `aborted` (stall check / watchdog), `rejected` |
| `commands/<name>/completedCount` | robot | increments each time a run ends |
| `commands/<name>/lastResult` | robot | human-readable outcome (for `aborted`, the watchdog's reason) |
| `commands/<name>/expected` | robot | JSON object of the end values the current run should reach |
| `cancelAllRequest` | client | request id; cancel everything |
| `state/<key>` | robot | telemetry (`drivetrain/x_meters`, `arm/angle_degrees`, `robot/enabled`, ...) |
| `state/avoidance_zones` | robot | JSON array of rectangles the robot must not enter: the four `field_edge_*` boundary zones, then user zones |
| `state/field_bounds` | robot | JSON `{min_x, min_y, max_x, max_y}` of the drivable area (field size from the season's AprilTag layout, less 0.5 m wall clearance) |

Only the client publishes the request-id topics, so there is no writer race on them: the earlier
boolean `run`/`cancel` flags were reset by the robot, which let a single request be consumed
twice. Request ids are time-based, so a restarted client keeps counting upwards; an id that is
not larger than the last one (or the first value the robot sees after booting) is adopted as the
baseline without scheduling anything.

Field frame: x/y are WPILib field coordinates in meters and heading 0° faces +x, 90° faces +y
(counter-clockwise positive). The tool descriptions and system prompt spell this out so the model
can turn zone coordinates into routes.

## Layout

```
src/robot_llm/
  nt_client.py   RobotClient: NT connection, manifest, run_command (records a trace, check-ins), get_state
  monitor.py     CommandTrace, client-side rules, expected-vs-actual comparison, tool-result formatting
  tools.py       manifest -> LangChain StructuredTools (+ get_robot_state, wait_for_command, cancel_command,
                 get_command_trace, cancel_all_commands, wait_seconds)
  agent.py       create_agent() wiring, system prompt, console rendering
  cli.py         interactive REPL / one-shot runner  (entry point: `robot-llm`)
  server.py      FastAPI WebSocket bridge for the web frontend  (entry point: `robot-llm-server`)
robot-llm-frontend/
  src/           Vite + React + MUI + zustand web UI: chat, running command, robot state, command list
ExampleRobotProject/src/main/java/frc/robot/
  llm/LlmCommands.java         registry + NT protocol; call periodic() from robotPeriodic()
  llm/LlmCommandBuilder.java   fluent registration API (params, timeout, track/expected/stallTimeout/watchdog)
  llm/LlmRun.java              per-run context handed to a watchdog (params, expected, start state, elapsed)
  llm/LlmWatchdog.java         robot-side health check interface
  llm/LlmParams.java           typed access to the parameters the client sent
  llm/ParamSpec.java           parameter description published in the manifest
  subsystems/                  simulated Drivetrain, Arm and Intake (no hardware needed)
  RobotContainer.java          example command registrations
ExampleRobotProject/src/test/java/frc/robot/llm/LlmCommandsTest.java   protocol tests under HAL sim
tests/           Python tests: monitor rules/formatting, and RobotClient against an in-process NT server
```

### Example commands exposed by the robot

| Tool | Parameters | Description |
| --- | --- | --- |
| `drive_distance` | `meters` (−10..10) | drive straight, signed distance |
| `turn_to_heading` | `degrees` (−180..180) | rotate to an absolute field heading (0 = +x, 90 = +y) |
| `turn_by` | `degrees` (−360..360) | rotate relative to the current heading |
| `drive_to_point` | `x`, `y` (inside `field_bounds`) | turn toward a field position, then drive to it; one call per waypoint |
| `stop_driving` | – | stop the drivetrain |
| `arm_to_angle` | `degrees` (0..120) | move the arm to an angle |
| `arm_to_preset` | `preset` ∈ stowed, intake, low, high | move the arm to a named position |
| `intake_game_piece` | – | lower arm and run rollers until a piece is held |
| `eject_game_piece` | – | reverse rollers briefly |
| `score` | `level` ∈ low, high | arm up → eject → stow (no-op if nothing held) |
| `say` | `message` | print to the robot console |
| `add_avoidance_zone` | `name`, `x1`, `y1`, `x2`, `y2` | forbid a rectangle given by two opposite corners |
| `clear_avoidance_zones` | – | remove user zones (the `field_edge_*` fence stays) |

Every drive command is rejected up front if its straight-line path would cross a zone, and
aborted by the robot watchdog if the robot enters one mid-run.

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

#### Monitoring a command

All of these are optional; commands without them still work, the model just gets less evidence.

```java
LlmCommands.register("drive_distance")
    ...
    .checkIn(5.0)                       // hand the trace back to the model every 5 s while running
    .track("drivetrain/x_meters", "drivetrain/y_meters", "drivetrain/heading_degrees")
    .expected(p -> Map.of(              // end values, published under commands/<name>/expected
        "drivetrain/x_meters", targetX(p), "drivetrain/y_meters", targetY(p)))
    .stallTimeout(1.0)                  // abort if no tracked value changes for 1 s
    .watchdog(run -> {                  // evaluated every loop; return a reason to abort
        double off = crossTrackError(run.startDouble("drivetrain/x_meters"), ...);
        return off > 0.15 ? String.format("%.2f m off the line", off) : null;
    })
    .command(p -> m_drivetrain.driveDistance(p.getDouble("meters")));
```

- `track` names the state keys that describe this command's progress. The client records them
  while the command runs; the robot uses them for the stall check.
- `expected` runs when the request is received (so it can read the current pose). Values are
  compared with the final state on the client using per-key tolerances (`_meters` 0.1,
  `_degrees` 3, heading wrap-aware).
- `watchdog` gets an `LlmRun` with the params, expected values, the tracked state at start and
  live access to `/LLM/state`. Returning a string cancels the command and reports it as
  `aborted` with that reason. See `RobotContainer` for straight-line and turn-in-place examples.

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
status. **Stop all** in the toolbar sends `cancelAllRequest`.

Bridge protocol (JSON over the WebSocket): server sends `hello`/`manifest` (command specs),
`snapshot` (robot connection, telemetry, per-command status — only when something changes),
`turn` (`start`/`end`), `message` (`assistant`, `tool_call`, `tool_result`) and `error`; the client
sends `chat {text}`, `cancel_all` and `cancel {name}`. See `src/robot_llm/server.py`.

## Tests

```sh
cd ExampleRobotProject && ./gradlew test    # Java
uv run pytest                               # Python
```

The Java tests run the robot code under HAL simulation and drive it through the NetworkTables
protocol (manifest contents, rejection while disabled, completion, cancellation, sequences,
expected values, watchdog and stall aborts, timeout reporting). The Python tests cover the
monitor rules and result formatting, and run `RobotClient` against an in-process NetworkTables
server that plays a small fake robot (traces, discrepancies, client-side aborts, check-ins).

## Links

- ntcore docs: https://robotpy.readthedocs.io/projects/robotpy/en/stable/api.html#ntcore-api
- langchain docs: https://docs.langchain.com/oss/python/langchain/overview
- wpilib docs: https://docs.wpilib.org/en/stable/index.html
