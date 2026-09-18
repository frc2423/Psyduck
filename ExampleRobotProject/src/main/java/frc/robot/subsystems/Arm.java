// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.llm.LlmCommands;

/** A simulated single-joint arm that slews toward a setpoint at a fixed rate. */
public class Arm extends SubsystemBase {
  public static final double kMinAngleDegrees = 0.0;
  public static final double kMaxAngleDegrees = 120.0;
  public static final double kSlewDegreesPerSecond = 60.0;
  private static final double kToleranceDegrees = 0.5;

  /** Named positions the LLM can ask for by name. */
  public enum Preset {
    STOWED(0.0),
    INTAKE(15.0),
    LOW(45.0),
    HIGH(100.0);

    public final double degrees;

    Preset(double degrees) {
      this.degrees = degrees;
    }

    public static String[] names() {
      Preset[] values = values();
      String[] names = new String[values.length];
      for (int i = 0; i < values.length; i++) {
        names[i] = values[i].name().toLowerCase();
      }
      return names;
    }
  }

  private double m_angleDegrees = Preset.STOWED.degrees;
  private double m_setpointDegrees = Preset.STOWED.degrees;

  public double getAngleDegrees() {
    return m_angleDegrees;
  }

  public boolean atSetpoint() {
    return Math.abs(m_setpointDegrees - m_angleDegrees) < kToleranceDegrees;
  }

  public void setSetpoint(double degrees) {
    m_setpointDegrees = MathUtil.clamp(degrees, kMinAngleDegrees, kMaxAngleDegrees);
  }

  /** Move to an angle and finish once there. Holds position afterwards. */
  public Command goToAngle(double degrees) {
    return runOnce(() -> setSetpoint(degrees))
        .andThen(run(() -> {}).until(this::atSetpoint))
        .withName("ArmToAngle");
  }

  /** Move to a named preset. Throws if the name is unknown so the LLM gets a clear error. */
  public Command goToPreset(String name) {
    Preset preset = Preset.valueOf(name.trim().toUpperCase());
    return goToAngle(preset.degrees).withName("ArmToPreset(" + preset.name() + ")");
  }

  @Override
  public void periodic() {
    double step = kSlewDegreesPerSecond * TimedRobot.kDefaultPeriod;
    double error = m_setpointDegrees - m_angleDegrees;
    m_angleDegrees += MathUtil.clamp(error, -step, step);

    LlmCommands.publishState("arm/angle_degrees", m_angleDegrees);
    LlmCommands.publishState("arm/setpoint_degrees", m_setpointDegrees);
    LlmCommands.publishState("arm/at_setpoint", atSetpoint());
  }
}
