// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.llm;

import java.util.List;

/**
 * Describes a single parameter of an LLM-triggerable command.
 *
 * <p>Published to NetworkTables as part of the manifest so the Python client can build a typed
 * tool schema for the language model.
 *
 * @param name parameter name; also the NetworkTables key under {@code params/}
 * @param type one of {@code double}, {@code integer}, {@code boolean}, {@code string}
 * @param description natural-language description shown to the model
 * @param min optional lower bound (numeric types only, may be null)
 * @param max optional upper bound (numeric types only, may be null)
 * @param choices optional allowed values (string type only, may be null)
 */
public record ParamSpec(
    String name, String type, String description, Double min, Double max, List<String> choices) {

  public static ParamSpec ofDouble(String name, String description, Double min, Double max) {
    return new ParamSpec(name, "double", description, min, max, null);
  }

  public static ParamSpec ofInteger(String name, String description, Double min, Double max) {
    return new ParamSpec(name, "integer", description, min, max, null);
  }

  public static ParamSpec ofBoolean(String name, String description) {
    return new ParamSpec(name, "boolean", description, null, null, null);
  }

  public static ParamSpec ofString(String name, String description, List<String> choices) {
    return new ParamSpec(name, "string", description, null, null, choices);
  }
}
