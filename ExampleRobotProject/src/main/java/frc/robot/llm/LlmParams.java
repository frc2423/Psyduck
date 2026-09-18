// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.llm;

import java.util.Map;

/**
 * Snapshot of the parameter values the client wrote before triggering a command.
 *
 * <p>Values are read from NetworkTables at the moment the trigger is seen, so a command factory
 * can safely capture them.
 */
public final class LlmParams {
  private final Map<String, Object> m_values;

  LlmParams(Map<String, Object> values) {
    m_values = values;
  }

  public double getDouble(String name) {
    Object v = m_values.get(name);
    return v instanceof Number n ? n.doubleValue() : 0.0;
  }

  public long getInteger(String name) {
    Object v = m_values.get(name);
    return v instanceof Number n ? n.longValue() : 0;
  }

  public boolean getBoolean(String name) {
    Object v = m_values.get(name);
    return v instanceof Boolean b && b;
  }

  public String getString(String name) {
    Object v = m_values.get(name);
    return v instanceof String s ? s : "";
  }

  @Override
  public String toString() {
    return m_values.toString();
  }
}
