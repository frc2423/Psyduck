// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.llm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.junit.jupiter.api.TestInstance.Lifecycle;

/**
 * Exercises the NetworkTables protocol end to end inside one JVM: the test plays the role of the
 * Python client by writing to the same {@link NetworkTableInstance} the robot code reads from.
 */
@TestInstance(Lifecycle.PER_CLASS)
class LlmCommandsTest {
  private static final NetworkTable kTable = NetworkTableInstance.getDefault().getTable("LLM");
  private static final NetworkTable kCommands = kTable.getSubTable("commands");
  private static final NetworkTable kState = kTable.getSubTable("state");

  @BeforeAll
  static void setUpAll() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    DriverStationSim.setDsAttached(true);
    // RobotContainer registers the example commands with the LlmCommands singleton.
    new RobotContainer();
  }

  @AfterAll
  static void tearDownAll() {
    SimHooks.resumeTiming();
    HAL.shutdown();
  }

  @BeforeEach
  void setUp() {
    CommandScheduler.getInstance().cancelAll();
    setEnabled(true);
    tick(1);
  }

  private static void setEnabled(boolean enabled) {
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
  }

  /** Run n robot loops: scheduler + LLM polling, advancing sim time by 20 ms each. */
  private static void tick(int n) {
    for (int i = 0; i < n; i++) {
      SimHooks.stepTiming(0.02);
      CommandScheduler.getInstance().run();
      LlmCommands.getInstance().periodic();
    }
  }

  private static long completedCount(String name) {
    return kCommands.getSubTable(name).getEntry("completedCount").getInteger(-1);
  }

  private static String status(String name) {
    return kCommands.getSubTable(name).getEntry("status").getString("?");
  }

  /** Trigger a command and tick until it reports completion or maxTicks elapse. */
  private static String runAndWait(String name, int maxTicks) {
    long before = completedCount(name);
    kCommands.getSubTable(name).getEntry("run").setBoolean(true);
    for (int i = 0; i < maxTicks && completedCount(name) == before; i++) {
      tick(1);
    }
    assertTrue(completedCount(name) > before, name + " never completed; status=" + status(name));
    return status(name);
  }

  @Test
  void manifestListsRegisteredCommands() throws Exception {
    String manifest = kTable.getEntry("manifest").getString("");
    JsonNode root = new ObjectMapper().readTree(manifest);
    assertEquals(1, root.get("version").asInt());

    JsonNode drive = null;
    for (JsonNode cmd : root.get("commands")) {
      if (cmd.get("name").asText().equals("drive_distance")) {
        drive = cmd;
      }
    }
    assertTrue(drive != null, "drive_distance missing from manifest: " + manifest);
    JsonNode meters = drive.get("parameters").get(0);
    assertEquals("meters", meters.get("name").asText());
    assertEquals("double", meters.get("type").asText());
    assertEquals(-10.0, meters.get("min").asDouble());
    assertEquals(10.0, meters.get("max").asDouble());
  }

  @Test
  void rejectsWhenDisabled() {
    setEnabled(false);
    tick(1);
    assertEquals("rejected", runAndWait("say", 5));
    assertEquals("disabled", kState.getEntry("robot/mode").getString(""));
  }

  @Test
  void instantCommandFinishes() {
    kCommands.getSubTable("say").getSubTable("params").getEntry("message").setString("hello");
    assertEquals("finished", runAndWait("say", 10));
  }

  @Test
  void driveDistanceMovesRobotAndFinishes() {
    double startX = kState.getEntry("drivetrain/x_meters").getDouble(0.0);
    kCommands.getSubTable("drive_distance").getSubTable("params").getEntry("meters").setDouble(1.0);
    assertEquals("finished", runAndWait("drive_distance", 500));
    double travelled = kState.getEntry("drivetrain/x_meters").getDouble(0.0) - startX;
    assertEquals(1.0, travelled, 0.05);
    tick(1); // subsystem telemetry is published one loop after the command ends
    assertEquals(false, kState.getEntry("drivetrain/moving").getBoolean(true));
  }

  @Test
  void cancelInterruptsRunningCommand() {
    kCommands.getSubTable("drive_distance").getSubTable("params").getEntry("meters").setDouble(8.0);
    long before = completedCount("drive_distance");
    kCommands.getSubTable("drive_distance").getEntry("run").setBoolean(true);
    tick(10);
    assertEquals("running", status("drive_distance"));

    kCommands.getSubTable("drive_distance").getEntry("cancel").setBoolean(true);
    tick(2);
    assertEquals(before + 1, completedCount("drive_distance"));
    assertEquals("interrupted", status("drive_distance"));
  }

  @Test
  void presetChoiceDrivesArmAndIntakeSequenceAcquiresPiece() {
    kCommands.getSubTable("arm_to_preset").getSubTable("params").getEntry("preset").setString("high");
    assertEquals("finished", runAndWait("arm_to_preset", 200));
    assertEquals(100.0, kState.getEntry("arm/angle_degrees").getDouble(0.0), 1.0);

    assertEquals("finished", runAndWait("intake_game_piece", 400));
    assertTrue(kState.getEntry("intake/has_game_piece").getBoolean(false));

    kCommands.getSubTable("score").getSubTable("params").getEntry("level").setString("low");
    assertEquals("finished", runAndWait("score", 500));
    assertEquals(false, kState.getEntry("intake/has_game_piece").getBoolean(true));
    assertEquals(0.0, kState.getEntry("arm/angle_degrees").getDouble(99.0), 1.0);
  }
}
