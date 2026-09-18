// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.llm.LlmCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import java.util.Set;

public class RobotContainer {
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake();

  public RobotContainer() {
    configureBindings();
    configureLlmCommands();
  }

  private void configureBindings() {}

  /**
   * Every command registered here becomes a tool the LLM can call. The names, descriptions and
   * parameter specs are published to NetworkTables and turned into tool schemas on the Python side,
   * so keep the descriptions precise about units and sign conventions.
   */
  private void configureLlmCommands() {
    // --- Drivetrain -------------------------------------------------------------------------
    LlmCommands.register("drive_distance")
        .description(
            "Drive the robot in a straight line along its current heading. Positive distance"
                + " drives forward, negative drives backward. Finishes when the distance is"
                + " reached.")
        .doubleParam("meters", "Signed distance to travel in meters.", -10.0, 10.0)
        .timeout(20.0)
        .command(p -> m_drivetrain.driveDistance(p.getDouble("meters")));

    LlmCommands.register("turn_to_heading")
        .description(
            "Rotate the robot in place to an absolute field heading. 0 degrees is the direction"
                + " the robot faced at startup; positive angles are counter-clockwise.")
        .doubleParam("degrees", "Target heading in degrees, -180 to 180.", -180.0, 180.0)
        .timeout(10.0)
        .command(p -> m_drivetrain.turnToAngle(p.getDouble("degrees")));

    LlmCommands.register("turn_by")
        .description(
            "Rotate the robot in place by a relative angle from its current heading. Positive"
                + " turns left (counter-clockwise), negative turns right.")
        .doubleParam("degrees", "Relative angle to rotate in degrees.", -360.0, 360.0)
        .timeout(10.0)
        .command(
            p ->
                Commands.defer(
                    () ->
                        m_drivetrain.turnToAngle(
                            m_drivetrain.getHeadingDegrees() + p.getDouble("degrees")),
                    Set.of(m_drivetrain)));

    LlmCommands.register("stop_driving")
        .description("Immediately stop all drivetrain motion.")
        .command(m_drivetrain::stopCommand);

    // --- Arm --------------------------------------------------------------------------------
    LlmCommands.register("arm_to_angle")
        .description("Move the arm to an absolute angle and hold it there. 0 is stowed flat.")
        .doubleParam(
            "degrees", "Target arm angle in degrees.", Arm.kMinAngleDegrees, Arm.kMaxAngleDegrees)
        .timeout(5.0)
        .command(p -> m_arm.goToAngle(p.getDouble("degrees")));

    LlmCommands.register("arm_to_preset")
        .description("Move the arm to one of its named preset positions.")
        .choiceParam("preset", "Which preset position to move to.", Arm.Preset.names())
        .timeout(5.0)
        .command(p -> m_arm.goToPreset(p.getString("preset")));

    // --- Intake -----------------------------------------------------------------------------
    LlmCommands.register("intake_game_piece")
        .description(
            "Lower the arm to the intake position and run the intake rollers until a game piece"
                + " is acquired.")
        .timeout(8.0)
        .command(
            p ->
                m_arm
                    .goToPreset("intake")
                    .andThen(m_intake.intakeGamePiece())
                    .withName("IntakeSequence"));

    LlmCommands.register("eject_game_piece")
        .description("Reverse the intake rollers briefly to release the held game piece.")
        .command(p -> m_intake.eject());

    // --- Composite routines -----------------------------------------------------------------
    LlmCommands.register("score")
        .description(
            "Score the currently held game piece: raise the arm to the chosen level, eject,"
                + " then stow the arm. Fails immediately if no game piece is held.")
        .choiceParam("level", "Scoring level to use.", "low", "high")
        .timeout(15.0)
        .command(
            p ->
                Commands.either(
                    m_arm
                        .goToPreset(p.getString("level"))
                        .andThen(m_intake.eject())
                        .andThen(m_arm.goToPreset("stowed")),
                    Commands.print("[score] no game piece held, nothing to score"),
                    m_intake::hasGamePiece));

    LlmCommands.register("say")
        .description("Print a message to the robot console. Useful for acknowledging the operator.")
        .stringParam("message", "The text to print.")
        .command(p -> Commands.print("[robot says] " + p.getString("message")));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
