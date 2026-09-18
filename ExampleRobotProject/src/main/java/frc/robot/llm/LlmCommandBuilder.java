// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.llm;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;
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
 *     .command(p -> drivetrain.driveDistance(p.getDouble("meters")));
 * }</pre>
 */
public final class LlmCommandBuilder {
  private final LlmCommands m_registry;
  private final String m_name;
  private String m_description = "";
  private double m_timeoutSeconds = 0.0;
  private final List<ParamSpec> m_params = new ArrayList<>();

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
    m_registry.add(new LlmCommandSpec(m_name, m_description, m_timeoutSeconds, m_params, factory));
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
      List<ParamSpec> params,
      Function<LlmParams, Command> factory) {}
}
