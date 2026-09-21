// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.llm;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Timer;
import java.util.Collections;
import java.util.Map;

/**
 * Context for one execution of an LLM command, handed to its {@link LlmWatchdog}.
 *
 * <p>Holds the parameters the client sent, the expected end values the command declared, and a
 * snapshot of the tracked state keys taken in the loop the command was scheduled, plus live access
 * to the {@code /LLM/state} table.
 */
public final class LlmRun {
  private final LlmParams m_params;
  private final Map<String, Object> m_expected;
  private final Map<String, Object> m_startState;
  private final double m_startTime;
  private final NetworkTable m_stateTable;

  LlmRun(
      LlmParams params,
      Map<String, Object> expected,
      Map<String, Object> startState,
      NetworkTable stateTable) {
    m_params = params;
    m_expected = Collections.unmodifiableMap(expected);
    m_startState = Collections.unmodifiableMap(startState);
    m_startTime = Timer.getFPGATimestamp();
    m_stateTable = stateTable;
  }

  public LlmParams params() {
    return m_params;
  }

  /** Seconds since the command was scheduled. */
  public double elapsedSeconds() {
    return Timer.getFPGATimestamp() - m_startTime;
  }

  /** The expected end values declared via {@code LlmCommandBuilder.expected}. */
  public Map<String, Object> expected() {
    return m_expected;
  }

  public double expectedDouble(String key) {
    return asDouble(m_expected.get(key));
  }

  /** Values of the tracked state keys when the command was scheduled. */
  public Map<String, Object> startState() {
    return m_startState;
  }

  public double startDouble(String key) {
    return asDouble(m_startState.get(key));
  }

  /** Current value of a {@code /LLM/state} key. */
  public double stateDouble(String key) {
    return m_stateTable.getEntry(key).getDouble(0.0);
  }

  public boolean stateBoolean(String key) {
    return m_stateTable.getEntry(key).getBoolean(false);
  }

  static double asDouble(Object value) {
    return value instanceof Number n ? n.doubleValue() : 0.0;
  }
}
