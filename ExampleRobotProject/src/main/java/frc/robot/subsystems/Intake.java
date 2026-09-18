// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.llm.LlmCommands;

/**
 * A simulated intake. Running the rollers forward "acquires" a game piece after a short delay;
 * running them in reverse ejects it.
 */
public class Intake extends SubsystemBase {
  private static final double kAcquireSeconds = 1.5;
  private static final double kEjectSeconds = 0.5;

  private double m_rollerSpeed; // -1..1, positive intakes
  private boolean m_hasGamePiece;
  private final Timer m_rollerTimer = new Timer();

  public boolean hasGamePiece() {
    return m_hasGamePiece;
  }

  public void setRollers(double speed) {
    if (speed != m_rollerSpeed) {
      m_rollerTimer.restart();
    }
    m_rollerSpeed = speed;
  }

  public void stop() {
    setRollers(0.0);
  }

  /** Run the rollers until a game piece is acquired. */
  public Command intakeGamePiece() {
    return run(() -> setRollers(1.0))
        .until(this::hasGamePiece)
        .finallyDo(this::stop)
        .withName("IntakeGamePiece");
  }

  /** Reverse the rollers briefly to eject whatever is held. */
  public Command eject() {
    return run(() -> setRollers(-1.0))
        .withTimeout(kEjectSeconds)
        .finallyDo(this::stop)
        .withName("Eject");
  }

  @Override
  public void periodic() {
    if (m_rollerSpeed > 0 && !m_hasGamePiece && m_rollerTimer.hasElapsed(kAcquireSeconds)) {
      m_hasGamePiece = true;
    } else if (m_rollerSpeed < 0 && m_hasGamePiece && m_rollerTimer.hasElapsed(kEjectSeconds / 2)) {
      m_hasGamePiece = false;
    }

    LlmCommands.publishState("intake/roller_speed", m_rollerSpeed);
    LlmCommands.publishState("intake/has_game_piece", m_hasGamePiece);
  }
}
