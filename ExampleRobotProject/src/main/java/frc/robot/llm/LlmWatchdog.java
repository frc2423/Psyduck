// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.llm;

/**
 * Robot-side health check evaluated every loop while an LLM command runs.
 *
 * <p>A watchdog runs at the robot loop rate, so it can react to a problem within one cycle
 * instead of waiting for the client to notice over NetworkTables. When it returns a non-null
 * reason the registry cancels the command and reports it as {@code aborted} with that reason.
 */
@FunctionalInterface
public interface LlmWatchdog {
  /** Return {@code null} if the run looks healthy, otherwise a short reason for aborting it. */
  String check(LlmRun run);
}
