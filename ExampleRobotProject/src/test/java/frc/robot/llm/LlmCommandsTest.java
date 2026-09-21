// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.llm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import java.util.Map;
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
  static void setUpAll() throws Exception {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    DriverStationSim.setDsAttached(true);
    // RobotContainer registers the example commands with the LlmCommands singleton.
    new RobotContainer();
    // A command that never makes progress, to exercise the stall check and timeout reporting.
    LlmCommands.register("test_idle")
        .timeout(3.0)
        .track("arm/angle_degrees")
        .stallTimeout(0.5)
        .expected(p -> Map.of("arm/angle_degrees", 42.0))
        .command(() -> Commands.idle());
    LlmCommands.register("test_idle_no_stall").timeout(0.5).command(() -> Commands.idle());

    // Like the Python client, publish a baseline request id for every command before sending
    // any real request: the robot adopts the first id it sees without running anything.
    tick(1);
    for (JsonNode cmd : new ObjectMapper().readTree(kTable.getEntry("manifest").getString("")).get("commands")) {
      NetworkTable table = kCommands.getSubTable(cmd.get("name").asText());
      table.getEntry("runRequest").setInteger(0);
      table.getEntry("cancelRequest").setInteger(0);
    }
    kTable.getEntry("cancelAllRequest").setInteger(0);
    tick(1);
  }

  private static long sRequestId = 0;

  /** Send a run request for {@code name} with a fresh id. */
  private static long trigger(String name) {
    long id = ++sRequestId;
    kCommands.getSubTable(name).getEntry("runRequest").setInteger(id);
    return id;
  }

  private static void cancel(String name) {
    kCommands.getSubTable(name).getEntry("cancelRequest").setInteger(++sRequestId);
  }

  private static int boundaryZoneCount() {
    return (int) LlmCommands.getAvoidanceZones().stream()
        .filter(z -> z.name().startsWith("field_edge_"))
        .count();
  }

  @AfterAll
  static void tearDownAll() {
    SimHooks.resumeTiming();
    HAL.shutdown();
  }

  @BeforeEach
  void setUp() {
    CommandScheduler.getInstance().cancelAll();
    LlmCommands.clearAvoidanceZones();
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
    trigger(name);
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
    assertEquals(3, root.get("version").asInt());

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

    assertEquals(5.0, drive.get("checkInSeconds").asDouble());
    assertEquals(1.0, drive.get("stallTimeoutSeconds").asDouble());
    assertTrue(drive.get("hasWatchdog").asBoolean());
    assertEquals("drivetrain/x_meters", drive.get("trackedState").get(0).asText());
  }

  private static String lastResult(String name) {
    return kCommands.getSubTable(name).getEntry("lastResult").getString("");
  }

  @Test
  void expectedValuesArePublishedWhenRunStarts() throws Exception {
    double startX = kState.getEntry("drivetrain/x_meters").getDouble(0.0);
    double headingRad = Math.toRadians(kState.getEntry("drivetrain/heading_degrees").getDouble(0.0));
    kCommands.getSubTable("drive_distance").getSubTable("params").getEntry("meters").setDouble(1.0);
    long id = trigger("drive_distance");
    tick(2);
    assertEquals("running", status("drive_distance"));
    assertEquals(id, kCommands.getSubTable("drive_distance").getEntry("runAck").getInteger(-1));

    JsonNode expected =
        new ObjectMapper()
            .readTree(kCommands.getSubTable("drive_distance").getEntry("expected").getString(""));
    assertEquals(startX + Math.cos(headingRad), expected.get("drivetrain/x_meters").asDouble(), 1e-6);
    assertTrue(expected.has("drivetrain/heading_degrees"));

    cancel("drive_distance");
    tick(2);
    assertEquals("interrupted", status("drive_distance"));
    assertEquals("command was cancelled", lastResult("drive_distance"));
  }

  @Test
  void requestIdIsHandledOnceAndStaleIdsAreIgnored() {
    kCommands.getSubTable("say").getSubTable("params").getEntry("message").setString("once");
    long before = completedCount("say");
    long id = trigger("say");
    // The same id stays published for many loops; it must not be consumed again.
    tick(10);
    assertEquals(before + 1, completedCount("say"));
    assertEquals(id, kCommands.getSubTable("say").getEntry("runAck").getInteger(-1));

    // A smaller id (a restarted client) becomes the new baseline without running anything.
    kCommands.getSubTable("say").getEntry("runRequest").setInteger(1);
    tick(5);
    assertEquals(before + 1, completedCount("say"));
    // ...and the next larger id runs again.
    kCommands.getSubTable("say").getEntry("runRequest").setInteger(2);
    tick(5);
    assertEquals(before + 2, completedCount("say"));
    sRequestId = Math.max(sRequestId, 2);
  }

  @Test
  void driveToPointTurnsThenDrivesToTarget() {
    double x = kState.getEntry("drivetrain/x_meters").getDouble(0.0);
    double y = kState.getEntry("drivetrain/y_meters").getDouble(0.0);
    // A target off to the side so the robot has to turn first.
    double tx = x + 1.0;
    double ty = y + 0.8;
    NetworkTable params = kCommands.getSubTable("drive_to_point").getSubTable("params");
    params.getEntry("x").setDouble(tx);
    params.getEntry("y").setDouble(ty);
    assertEquals("finished", runAndWait("drive_to_point", 1000), lastResult("drive_to_point"));
    assertEquals(tx, kState.getEntry("drivetrain/x_meters").getDouble(0.0), 0.05);
    assertEquals(ty, kState.getEntry("drivetrain/y_meters").getDouble(0.0), 0.05);
    double expectedHeading = Math.toDegrees(Math.atan2(0.8, 1.0));
    assertEquals(expectedHeading, kState.getEntry("drivetrain/heading_degrees").getDouble(0.0), 2.0);

    // Face +x again so later straight-drive tests keep heading down the field.
    kCommands.getSubTable("turn_to_heading").getSubTable("params").getEntry("degrees").setDouble(0.0);
    assertEquals("finished", runAndWait("turn_to_heading", 500));
  }

  @Test
  void driveToPointRejectedWhenLegCrossesZone() {
    double x = kState.getEntry("drivetrain/x_meters").getDouble(0.0);
    double y = kState.getEntry("drivetrain/y_meters").getDouble(0.0);
    LlmCommands.addAvoidanceZone("block", x + 0.8, y - 0.5, x + 1.2, y + 0.5);
    NetworkTable params = kCommands.getSubTable("drive_to_point").getSubTable("params");
    params.getEntry("x").setDouble(x + 2.0);
    params.getEntry("y").setDouble(y);
    assertEquals("rejected", runAndWait("drive_to_point", 5));
    assertTrue(lastResult("drive_to_point").contains("block"), lastResult("drive_to_point"));
  }

  @Test
  void fieldBoundsArePublishedAndFenceTheField() throws Exception {
    JsonNode bounds =
        new ObjectMapper().readTree(kState.getEntry("field_bounds").getString(""));
    assertEquals(0.5, bounds.get("min_x").asDouble(), 1e-9);
    assertEquals(0.5, bounds.get("min_y").asDouble(), 1e-9);
    assertTrue(bounds.get("max_x").asDouble() > 10.0, bounds.toString());
    assertTrue(bounds.get("max_y").asDouble() > 5.0, bounds.toString());
    assertEquals(4, boundaryZoneCount());

    // Backing straight out of the field (heading ~0, x < 10) crosses field_edge_min_x.
    double x = kState.getEntry("drivetrain/x_meters").getDouble(0.0);
    assertTrue(x < 10.0, "robot at x=" + x);
    kCommands.getSubTable("drive_distance").getSubTable("params").getEntry("meters").setDouble(-10.0);
    assertEquals("rejected", runAndWait("drive_distance", 5));
    assertTrue(lastResult("drive_distance").contains("field_edge_min_x"), lastResult("drive_distance"));
    assertEquals(x, kState.getEntry("drivetrain/x_meters").getDouble(99.0), 1e-6);
  }

  @Test
  void watchdogAbortsDriveThatEntersZoneAddedMidRun() {
    double x = kState.getEntry("drivetrain/x_meters").getDouble(0.0);
    double y = kState.getEntry("drivetrain/y_meters").getDouble(0.0);
    double headingRad = Math.toRadians(kState.getEntry("drivetrain/heading_degrees").getDouble(0.0));

    kCommands.getSubTable("drive_distance").getSubTable("params").getEntry("meters").setDouble(3.0);
    long before = completedCount("drive_distance");
    trigger("drive_distance");
    tick(10);
    assertEquals("running", status("drive_distance"));

    // Drop a zone 1 m ahead, in the path of the drive that is already under way.
    double cx = x + 1.0 * Math.cos(headingRad);
    double cy = y + 1.0 * Math.sin(headingRad);
    LlmCommands.addAvoidanceZone("surprise", cx - 0.3, cy + 0.3, cx + 0.3, cy - 0.3);

    for (int i = 0; i < 300 && completedCount("drive_distance") == before; i++) {
      tick(1);
    }
    assertEquals("aborted", status("drive_distance"));
    assertTrue(lastResult("drive_distance").contains("surprise"), lastResult("drive_distance"));
    // Aborted well before the 3 m target.
    double travelled = kState.getEntry("drivetrain/x_meters").getDouble(0.0) - x;
    assertTrue(Math.abs(travelled) < 1.5, "travelled " + travelled);
  }

  @Test
  void stallCheckAbortsCommandThatMakesNoProgress() {
    assertEquals("aborted", runAndWait("test_idle", 100));
    assertTrue(lastResult("test_idle").startsWith("no progress on"), lastResult("test_idle"));
    assertTrue(
        kCommands.getSubTable("test_idle").getEntry("expected").getString("").contains("42"));
  }

  @Test
  void timeoutIsReportedAsSuch() {
    assertEquals("interrupted", runAndWait("test_idle_no_stall", 100));
    assertTrue(lastResult("test_idle_no_stall").startsWith("timed out"), lastResult("test_idle_no_stall"));
  }

  @Test
  void disablingMidRunIsReported() {
    kCommands.getSubTable("drive_distance").getSubTable("params").getEntry("meters").setDouble(5.0);
    trigger("drive_distance");
    tick(5);
    assertEquals("running", status("drive_distance"));
    setEnabled(false);
    tick(2);
    assertEquals("interrupted", status("drive_distance"));
    assertTrue(lastResult("drive_distance").contains("disabled"), lastResult("drive_distance"));
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
    trigger("drive_distance");
    tick(10);
    assertEquals("running", status("drive_distance"));

    cancel("drive_distance");
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

  @Test
  void avoidanceZoneIsPublishedToState() throws Exception {
    ObjectMapper mapper = new ObjectMapper();
    int boundary = boundaryZoneCount();
    JsonNode zones = mapper.readTree(kState.getEntry("avoidance_zones").getString(""));
    assertEquals(boundary, zones.size());
    assertEquals("field_edge_min_x", zones.get(0).get("name").asText());

    // Corners given in any order; the published zone is normalised to min/max.
    LlmCommands.addAvoidanceZone("pit", 1.0, 3.0, 2.0, 2.0);
    zones = mapper.readTree(kState.getEntry("avoidance_zones").getString(""));
    assertEquals(boundary + 1, zones.size());
    JsonNode pit = zones.get(boundary);
    assertEquals("pit", pit.get("name").asText());
    assertEquals(1.0, pit.get("min_x").asDouble());
    assertEquals(2.0, pit.get("min_y").asDouble());
    assertEquals(2.0, pit.get("max_x").asDouble());
    assertEquals(3.0, pit.get("max_y").asDouble());

    // Clearing removes user zones but keeps the field fence.
    LlmCommands.clearAvoidanceZones();
    zones = mapper.readTree(kState.getEntry("avoidance_zones").getString(""));
    assertEquals(boundary, zones.size());
  }

  @Test
  void addAvoidanceZoneCommandUpdatesState() throws Exception {
    NetworkTable params = kCommands.getSubTable("add_avoidance_zone").getSubTable("params");
    params.getEntry("name").setString("charging_station");
    params.getEntry("x1").setDouble(4.0);
    params.getEntry("y1").setDouble(1.0);
    params.getEntry("x2").setDouble(5.0);
    params.getEntry("y2").setDouble(0.0);
    int boundary = boundaryZoneCount();
    assertEquals("finished", runAndWait("add_avoidance_zone", 10));

    JsonNode zones = new ObjectMapper().readTree(kState.getEntry("avoidance_zones").getString(""));
    assertEquals("charging_station", zones.get(boundary).get("name").asText());
    assertEquals(boundary + 1, LlmCommands.getAvoidanceZones().size());

    assertEquals("finished", runAndWait("clear_avoidance_zones", 10));
    assertEquals(boundary, LlmCommands.getAvoidanceZones().size());
  }

  @Test
  void driveDistanceRejectedWhenPathCrossesZone() {
    // Place a zone directly ahead of wherever the robot currently is, 2-3 m along its heading.
    double x = kState.getEntry("drivetrain/x_meters").getDouble(0.0);
    double y = kState.getEntry("drivetrain/y_meters").getDouble(0.0);
    double headingRad = Math.toRadians(kState.getEntry("drivetrain/heading_degrees").getDouble(0.0));
    double cx = x + 2.5 * Math.cos(headingRad);
    double cy = y + 2.5 * Math.sin(headingRad);
    LlmCommands.addAvoidanceZone("wall", cx - 0.5, cy + 0.5, cx + 0.5, cy - 0.5);

    kCommands.getSubTable("drive_distance").getSubTable("params").getEntry("meters").setDouble(5.0);
    assertEquals("rejected", runAndWait("drive_distance", 5));
    String result = kCommands.getSubTable("drive_distance").getEntry("lastResult").getString("");
    assertTrue(result.contains("wall"), "expected zone name in result: " + result);
    // The robot never moved.
    assertEquals(x, kState.getEntry("drivetrain/x_meters").getDouble(99.0), 1e-6);

    // A short drive that stops before the zone is still allowed.
    kCommands.getSubTable("drive_distance").getSubTable("params").getEntry("meters").setDouble(1.0);
    assertEquals("finished", runAndWait("drive_distance", 500));
  }

  @Test
  void zoneGeometry() {
    AvoidanceZone zone = new AvoidanceZone("z", 1.0, 1.0, 2.0, 2.0);
    assertTrue(zone.contains(new Translation2d(1.5, 1.5)));
    assertFalse(zone.contains(new Translation2d(0.5, 1.5)));
    // Crosses the box diagonally.
    assertTrue(zone.intersectsSegment(new Translation2d(0.0, 0.0), new Translation2d(3.0, 3.0)));
    // Passes alongside without touching.
    assertFalse(zone.intersectsSegment(new Translation2d(0.0, 0.5), new Translation2d(3.0, 0.5)));
    // Axis-aligned segment through the middle (zero dy).
    assertTrue(zone.intersectsSegment(new Translation2d(0.0, 1.5), new Translation2d(3.0, 1.5)));
    // Stops short of the box.
    assertFalse(zone.intersectsSegment(new Translation2d(0.0, 1.5), new Translation2d(0.9, 1.5)));
    // Starts inside the box.
    assertTrue(zone.intersectsSegment(new Translation2d(1.5, 1.5), new Translation2d(5.0, 1.5)));
  }
}
