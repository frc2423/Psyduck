// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.llm.LlmCommands;

/**
 * A kinematics-only differential drivetrain.
 *
 * <p>There is no hardware here; {@link #periodic()} integrates the commanded velocities so the
 * example can be driven entirely in simulation.
 */
public class Drivetrain extends SubsystemBase {
  public static final double kMaxSpeedMetersPerSecond = 1.0;
  public static final double kMaxTurnDegreesPerSecond = 90.0;
  private static final double kDistanceToleranceMeters = 0.02;
  private static final double kAngleToleranceDegrees = 1.0;
  /** Tighter alignment before a point-to-point drive, so a long leg does not miss the target. */
  private static final double kPointHeadingToleranceDegrees = 0.3;
  /** Slowest in-place turn rate; keeps small corrections from stalling short of the target. */
  private static final double kMinTurnDegreesPerSecond = 10.0;

  private Pose2d m_pose = new Pose2d();
  private double m_speed; // m/s, forward positive
  private double m_turnRate; // deg/s, CCW positive

  /** Field widget for dashboards (Glass, Shuffleboard, Elastic, AdvantageScope). */
  private final Field2d m_field = new Field2d();

  public Drivetrain() {
    SmartDashboard.putData("Field", m_field);
    m_field.setRobotPose(m_pose);
  }

  /** Directly set chassis velocities. */
  public void drive(double speedMetersPerSecond, double turnDegreesPerSecond) {
    m_speed = MathUtil.clamp(speedMetersPerSecond, -kMaxSpeedMetersPerSecond, kMaxSpeedMetersPerSecond);
    m_turnRate = MathUtil.clamp(turnDegreesPerSecond, -kMaxTurnDegreesPerSecond, kMaxTurnDegreesPerSecond);
  }

  public void stop() {
    drive(0.0, 0.0);
  }

  public Pose2d getPose() {
    return m_pose;
  }

  /** Teleport the simulated robot, e.g. to a legal starting position inside the field. */
  public void resetPose(Pose2d pose) {
    m_pose = pose;
    m_field.setRobotPose(m_pose);
  }

  public double getHeadingDegrees() {
    return m_pose.getRotation().getDegrees();
  }

  /** Drive straight for a signed distance in meters. */
  public Command driveDistance(double meters) {
    final double direction = Math.signum(meters);
    final double target = Math.abs(meters);
    final double[] travelled = {0.0};
    final Pose2d[] start = {null};

    return runOnce(() -> start[0] = m_pose)
        .andThen(
            run(
                () -> {
                  travelled[0] = m_pose.getTranslation().getDistance(start[0].getTranslation());
                  double remaining = target - travelled[0];
                  // Slow down over the last 0.5 m so we settle inside the tolerance.
                  double speed = MathUtil.clamp(remaining * 2.0, 0.1, kMaxSpeedMetersPerSecond);
                  drive(direction * speed, 0.0);
                }))
        .until(() -> travelled[0] >= target - kDistanceToleranceMeters)
        .finallyDo(this::stop)
        .withName("DriveDistance");
  }

  /** Rotate in place to an absolute field heading (degrees, CCW positive). */
  public Command turnToAngle(double degrees) {
    return run(() -> drive(0.0, turnRateFor(headingError(degrees))))
        .until(() -> Math.abs(headingError(degrees)) < kAngleToleranceDegrees)
        .finallyDo(this::stop)
        .withName("TurnToAngle");
  }

  /**
   * Turn in place to face {@code target}, then drive straight to it. The heading is re-aimed at
   * the target every loop while driving, so the robot converges on the point rather than on a
   * dead-reckoned distance.
   */
  public Command driveToPoint(Translation2d target) {
    final boolean[] aligned = {false};
    return runOnce(() -> aligned[0] = false)
        .andThen(
            run(
                () -> {
                  Translation2d delta = target.minus(m_pose.getTranslation());
                  double error = headingError(delta.getAngle().getDegrees());
                  if (!aligned[0]) {
                    aligned[0] = Math.abs(error) < kPointHeadingToleranceDegrees;
                    drive(0.0, turnRateFor(error));
                    return;
                  }
                  double remaining = delta.getNorm();
                  double speed = MathUtil.clamp(remaining * 2.0, 0.1, kMaxSpeedMetersPerSecond);
                  // Gentle steering only; close to the target the bearing swings wildly.
                  double steer = remaining > 0.1 ? MathUtil.clamp(error * 3.0, -15.0, 15.0) : 0.0;
                  drive(speed, steer);
                }))
        .until(() -> target.getDistance(m_pose.getTranslation()) < kDistanceToleranceMeters)
        .finallyDo(this::stop)
        .withName("DriveToPoint");
  }

  /** Signed shortest rotation from the current heading to {@code degrees}. */
  private double headingError(double degrees) {
    return MathUtil.inputModulus(degrees - getHeadingDegrees(), -180.0, 180.0);
  }

  /** Proportional in-place turn rate with a floor so the last degree does not take forever. */
  private static double turnRateFor(double error) {
    double rate = MathUtil.clamp(error * 3.0, -kMaxTurnDegreesPerSecond, kMaxTurnDegreesPerSecond);
    if (Math.abs(rate) < kMinTurnDegreesPerSecond) {
      rate = Math.copySign(kMinTurnDegreesPerSecond, error);
    }
    return rate;
  }

  /** Immediately stop and remain stopped. */
  public Command stopCommand() {
    return runOnce(this::stop).withName("StopDrive");
  }

  @Override
  public void periodic() {
    double dt = TimedRobot.kDefaultPeriod;
    Rotation2d heading = m_pose.getRotation();
    double dx = m_speed * heading.getCos() * dt;
    double dy = m_speed * heading.getSin() * dt;
    double dTheta = Math.toRadians(m_turnRate) * dt;
    m_pose =
        new Pose2d(
            m_pose.getX() + dx, m_pose.getY() + dy, heading.plus(Rotation2d.fromRadians(dTheta)));

    m_field.setRobotPose(m_pose);

    LlmCommands.publishState("drivetrain/x_meters", m_pose.getX());
    LlmCommands.publishState("drivetrain/y_meters", m_pose.getY());
    LlmCommands.publishState("drivetrain/heading_degrees", getHeadingDegrees());
    LlmCommands.publishState("drivetrain/speed_mps", m_speed);
    LlmCommands.publishState("drivetrain/moving", m_speed != 0.0 || m_turnRate != 0.0);
  }
}
