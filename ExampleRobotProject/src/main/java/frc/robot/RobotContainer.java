// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.llm.AvoidanceZone;
import frc.robot.llm.LlmCommands;
import frc.robot.llm.LlmRun;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import java.util.Map;
import java.util.Set;

public class RobotContainer {
  /** How far a straight drive may wander off its intended line before the watchdog aborts it. */
  private static final double kMaxCrossTrackMeters = 0.15;
  /** How far the heading may drift during a straight drive. */
  private static final double kMaxHeadingDriftDegrees = 10.0;
  /** How far the robot may translate while it is supposed to be turning in place. */
  private static final double kMaxTurnDriftMeters = 0.1;

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
                + " reached. Rejected if the straight-line path would cross an avoidance zone;"
                + " check avoidance_zones in the robot state and route around them.")
        .doubleParam("meters", "Signed distance to travel in meters.", -10.0, 10.0)
        .timeout(20.0)
        .checkIn(5.0)
        .track(
            "drivetrain/x_meters",
            "drivetrain/y_meters",
            "drivetrain/heading_degrees",
            "drivetrain/speed_mps",
            "drivetrain/moving")
        .expected(p -> expectedPoseAfterDriving(p.getDouble("meters")))
        .stallTimeout(1.0)
        .watchdog(this::straightDriveWatchdog)
        .command(
            p -> {
              requireClearStraightPath(p.getDouble("meters"));
              return m_drivetrain.driveDistance(p.getDouble("meters"));
            });

    LlmCommands.register("turn_to_heading")
        .description(
            "Rotate the robot in place to an absolute field heading. 0 degrees is the direction"
                + " the robot faced at startup; positive angles are counter-clockwise.")
        .doubleParam("degrees", "Target heading in degrees, -180 to 180.", -180.0, 180.0)
        .timeout(10.0)
        .track("drivetrain/heading_degrees", "drivetrain/x_meters", "drivetrain/y_meters")
        .expected(p -> expectedHeading(p.getDouble("degrees")))
        .stallTimeout(1.0)
        .watchdog(this::turnInPlaceWatchdog)
        .command(p -> m_drivetrain.turnToAngle(p.getDouble("degrees")));

    LlmCommands.register("turn_by")
        .description(
            "Rotate the robot in place by a relative angle from its current heading. Positive"
                + " turns left (counter-clockwise), negative turns right.")
        .doubleParam("degrees", "Relative angle to rotate in degrees.", -360.0, 360.0)
        .timeout(10.0)
        .track("drivetrain/heading_degrees", "drivetrain/x_meters", "drivetrain/y_meters")
        .expected(p -> expectedHeading(m_drivetrain.getHeadingDegrees() + p.getDouble("degrees")))
        .stallTimeout(1.0)
        .watchdog(this::turnInPlaceWatchdog)
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
        .track("arm/angle_degrees", "arm/at_setpoint")
        .expected(p -> Map.of("arm/angle_degrees", clampArm(p.getDouble("degrees"))))
        .stallTimeout(1.0)
        .command(p -> m_arm.goToAngle(p.getDouble("degrees")));

    LlmCommands.register("arm_to_preset")
        .description("Move the arm to one of its named preset positions.")
        .choiceParam("preset", "Which preset position to move to.", Arm.Preset.names())
        .timeout(5.0)
        .track("arm/angle_degrees", "arm/at_setpoint")
        .expected(p -> Map.of("arm/angle_degrees", Arm.Preset.parse(p.getString("preset")).degrees))
        .stallTimeout(1.0)
        .command(p -> m_arm.goToPreset(p.getString("preset")));

    // --- Intake -----------------------------------------------------------------------------
    LlmCommands.register("intake_game_piece")
        .description(
            "Lower the arm to the intake position and run the intake rollers until a game piece"
                + " is acquired.")
        .timeout(8.0)
        .track("arm/angle_degrees", "intake/roller_speed", "intake/has_game_piece")
        .expected(p -> Map.of("intake/has_game_piece", true))
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
        .track("arm/angle_degrees", "intake/roller_speed", "intake/has_game_piece")
        .expected(
            p ->
                Map.of(
                    "intake/has_game_piece", false,
                    "arm/angle_degrees", Arm.Preset.STOWED.degrees))
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

    // --- Avoidance zones ------------------------------------------------------------------------
    LlmCommands.register("add_avoidance_zone")
        .description(
            "Define a rectangular area of the field, in field coordinates (meters), that the robot"
                + " must not enter. Drive commands whose path would cross it are rejected. The"
                + " current zones are listed under avoidance_zones in the robot state.")
        .stringParam("name", "Short label for the zone, e.g. charging_station.")
        .doubleParam("top_left_x", "X coordinate of the top-left corner in meters.")
        .doubleParam("top_left_y", "Y coordinate of the top-left corner in meters.")
        .doubleParam("bottom_right_x", "X coordinate of the bottom-right corner in meters.")
        .doubleParam("bottom_right_y", "Y coordinate of the bottom-right corner in meters.")
        .command(
            p ->
                Commands.runOnce(
                    () ->
                        LlmCommands.addAvoidanceZone(
                            p.getString("name"),
                            p.getDouble("top_left_x"),
                            p.getDouble("top_left_y"),
                            p.getDouble("bottom_right_x"),
                            p.getDouble("bottom_right_y"))));

    LlmCommands.register("clear_avoidance_zones")
        .description("Remove every avoidance zone.")
        .command(() -> Commands.runOnce(LlmCommands::clearAvoidanceZones));

    LlmCommands.addAvoidanceZone("zone1", 4.039778, 6.753221, 5.200448, 1.327091);
  }

  /**
   * Throws if driving {@code meters} straight along the current heading would cross an avoidance
   * zone. Called from the command factory so the request is reported as {@code rejected} rather
   * than starting a motion that has to be aborted.
   */
  private void requireClearStraightPath(double meters) {
    Pose2d pose = m_drivetrain.getPose();
    Translation2d start = pose.getTranslation();
    Translation2d end = start.plus(new Translation2d(meters, pose.getRotation()));
    AvoidanceZone blocked = LlmCommands.findZoneCrossedBy(start, end);
    if (blocked != null) {
      throw new IllegalStateException(
          String.format(
              "path from (%.2f, %.2f) to (%.2f, %.2f) would enter avoidance zone '%s'",
              start.getX(), start.getY(), end.getX(), end.getY(), blocked.name()));
    }
  }

  /** Pose the robot should end at after driving {@code meters} straight from where it is now. */
  private Map<String, Object> expectedPoseAfterDriving(double meters) {
    Pose2d pose = m_drivetrain.getPose();
    Translation2d end = pose.getTranslation().plus(new Translation2d(meters, pose.getRotation()));
    return Map.of(
        "drivetrain/x_meters", end.getX(),
        "drivetrain/y_meters", end.getY(),
        "drivetrain/heading_degrees", pose.getRotation().getDegrees());
  }

  private static Map<String, Object> expectedHeading(double degrees) {
    return Map.of("drivetrain/heading_degrees", MathUtil.inputModulus(degrees, -180.0, 180.0));
  }

  private static double clampArm(double degrees) {
    return MathUtil.clamp(degrees, Arm.kMinAngleDegrees, Arm.kMaxAngleDegrees);
  }

  /**
   * Aborts a straight drive that leaves its intended line, drifts in heading, or enters an
   * avoidance zone (for example one added while it was moving).
   */
  private String straightDriveWatchdog(LlmRun run) {
    Pose2d pose = m_drivetrain.getPose();
    Translation2d start =
        new Translation2d(
            run.startDouble("drivetrain/x_meters"), run.startDouble("drivetrain/y_meters"));
    Translation2d end =
        new Translation2d(
            run.expectedDouble("drivetrain/x_meters"), run.expectedDouble("drivetrain/y_meters"));
    double crossTrack = distanceFromLine(pose.getTranslation(), start, end);
    if (crossTrack > kMaxCrossTrackMeters) {
      return String.format(
          "left the straight-line path: %.2f m off the line from (%.2f, %.2f) to (%.2f, %.2f)",
          crossTrack, start.getX(), start.getY(), end.getX(), end.getY());
    }
    double headingDrift =
        MathUtil.inputModulus(
            pose.getRotation().getDegrees() - run.expectedDouble("drivetrain/heading_degrees"),
            -180.0,
            180.0);
    if (Math.abs(headingDrift) > kMaxHeadingDriftDegrees) {
      return String.format("heading drifted %.1f degrees during a straight drive", headingDrift);
    }
    return zoneWatchdog(pose.getTranslation());
  }

  /** Aborts an in-place turn if the robot translates, or if it somehow enters a zone. */
  private String turnInPlaceWatchdog(LlmRun run) {
    Pose2d pose = m_drivetrain.getPose();
    Translation2d start =
        new Translation2d(
            run.startDouble("drivetrain/x_meters"), run.startDouble("drivetrain/y_meters"));
    double drift = pose.getTranslation().getDistance(start);
    if (drift > kMaxTurnDriftMeters) {
      return String.format("robot translated %.2f m while it should be turning in place", drift);
    }
    return zoneWatchdog(pose.getTranslation());
  }

  private static String zoneWatchdog(Translation2d position) {
    for (AvoidanceZone zone : LlmCommands.getAvoidanceZones()) {
      if (zone.contains(position)) {
        return String.format(
            "entered avoidance zone '%s' at (%.2f, %.2f)",
            zone.name(), position.getX(), position.getY());
      }
    }
    return null;
  }

  /** Perpendicular distance from {@code point} to the infinite line through a and b. */
  private static double distanceFromLine(Translation2d point, Translation2d a, Translation2d b) {
    double dx = b.getX() - a.getX();
    double dy = b.getY() - a.getY();
    double length = Math.hypot(dx, dy);
    if (length < 1e-6) {
      return point.getDistance(a);
    }
    return Math.abs(dy * (point.getX() - a.getX()) - dx * (point.getY() - a.getY())) / length;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
