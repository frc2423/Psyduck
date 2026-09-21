// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.llm;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * Fluent builder for exposing a command to the LLM client.
 *
 * <pre>{@code
 * LlmCommands.register("drive_distance")
 *     .description("Drive straight for a distance in meters.")
 *     .doubleParam("meters", "Distance to drive; negative drives backwards", -5.0, 5.0)
 *     .timeout(15.0)
 *     .track("drivetrain/x_meters", "drivetrain/y_meters")
 *     .expected(p -> Map.of("drivetrain/x_meters", targetX(p)))
 *     .stallTimeout(1.0)
 *     .watchdog(run -> crossTrackError(run) > 0.2 ? "left the straight-line path" : null)
 *     .command(p -> drivetrain.driveDistance(p.getDouble("meters")));
 * }</pre>
 *
 * <p>The monitoring hooks ({@link #track}, {@link #expected}, {@link #stallTimeout}, {@link
 * #watchdog}) are optional. Tracked keys tell the client which telemetry to record while the
 * command runs; expected values and the watchdog let the robot itself judge whether a run is going
 * to plan.
 */
public final class LlmCommandBuilder {
  private final LlmCommands m_registry;
  private final String m_name;
  private String m_description = "";
  private double m_timeoutSeconds = 0.0;
  private double m_checkInSeconds = 0.0;
  private double m_stallTimeoutSeconds = 0.0;
  private final List<ParamSpec> m_params = new ArrayList<>();
  private final List<String> m_trackedState = new ArrayList<>();
  private Function<LlmParams, Map<String, Object>> m_expected = p -> Map.of();
  private LlmWatchdog m_watchdog;

  LlmCommandBuilder(LlmCommands registry, String name) {
    m_registry = registry;
    m_name = name;
  }

  /** Natural-language description of what the command does. Shown to the model. */
  public LlmCommandBuilder description(String description) {
    m_description = description;
    return this;
  }

  /** Maximum run time in seconds before the command is cancelled. 0 disables the timeout. */
  public LlmCommandBuilder timeout(double seconds) {
    m_timeoutSeconds = seconds;
    return this;
  }

  /**
   * How often (seconds) the client should pause waiting and hand the trace so far back to the
   * model so it can decide to keep waiting or cancel. 0 uses the client's default.
   */
  public LlmCommandBuilder checkIn(double seconds) {
    m_checkInSeconds = seconds;
    return this;
  }

  /**
   * State keys (under {@code /LLM/state}) that describe this command's progress. They are
   * snapshotted when the run starts, sampled by the client while it runs, and used by {@link
   * #stallTimeout} to detect a run that has stopped making progress.
   */
  public LlmCommandBuilder track(String... stateKeys) {
    m_trackedState.addAll(List.of(stateKeys));
    return this;
  }

  /**
   * End values the command is expected to reach, keyed by state key. Evaluated when the run is
   * scheduled and published under {@code commands/<name>/expected} so both the robot watchdog and
   * the client can compare progress against them. Values may be numbers, booleans or strings.
   * Throwing rejects the run.
   */
  public LlmCommandBuilder expected(Function<LlmParams, Map<String, Object>> expected) {
    m_expected = expected;
    return this;
  }

  /**
   * Abort the run if none of the tracked numeric state keys changes for this many seconds. 0
   * disables the check. Only use on commands that are expected to move something continuously.
   */
  public LlmCommandBuilder stallTimeout(double seconds) {
    m_stallTimeoutSeconds = seconds;
    return this;
  }

  /** Robot-side health check evaluated every loop while the command runs. */
  public LlmCommandBuilder watchdog(LlmWatchdog watchdog) {
    m_watchdog = watchdog;
    return this;
  }

  public LlmCommandBuilder doubleParam(String name, String description) {
    return doubleParam(name, description, null, null);
  }

  public LlmCommandBuilder doubleParam(String name, String description, Double min, Double max) {
    m_params.add(ParamSpec.ofDouble(name, description, min, max));
    return this;
  }

  public LlmCommandBuilder integerParam(String name, String description) {
    return integerParam(name, description, null, null);
  }

  public LlmCommandBuilder integerParam(String name, String description, Double min, Double max) {
    m_params.add(ParamSpec.ofInteger(name, description, min, max));
    return this;
  }

  public LlmCommandBuilder booleanParam(String name, String description) {
    m_params.add(ParamSpec.ofBoolean(name, description));
    return this;
  }

  public LlmCommandBuilder stringParam(String name, String description) {
    m_params.add(ParamSpec.ofString(name, description, null));
    return this;
  }

  /** A string parameter restricted to a fixed set of values. */
  public LlmCommandBuilder choiceParam(String name, String description, String... choices) {
    m_params.add(ParamSpec.ofString(name, description, List.of(choices)));
    return this;
  }

  /** Finish registration with a factory that builds the command from the supplied parameters. */
  public void command(Function<LlmParams, Command> factory) {
    m_registry.add(
        new LlmCommandSpec(
            m_name,
            m_description,
            m_timeoutSeconds,
            m_checkInSeconds,
            m_stallTimeoutSeconds,
            m_params,
            List.copyOf(m_trackedState),
            m_expected,
            m_watchdog,
            factory));
  }

  /**
   * Finish registration with a parameterless command supplier. A supplier (rather than a single
   * {@link Command} instance) is required because each run is wrapped in a new composition, and
   * the scheduler forbids re-composing the same instance.
   */
  public void command(Supplier<Command> supplier) {
    command(p -> supplier.get());
  }

  /** Internal record describing a registered command. */
  record LlmCommandSpec(
      String name,
      String description,
      double timeoutSeconds,
      double checkInSeconds,
      double stallTimeoutSeconds,
      List<ParamSpec> params,
      List<String> trackedState,
      Function<LlmParams, Map<String, Object>> expected,
      LlmWatchdog watchdog,
      Function<LlmParams, Command> factory) {}
}
