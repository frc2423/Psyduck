// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.llm;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.llm.LlmCommandBuilder.LlmCommandSpec;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Exposes command-based framework commands to an external LLM client over NetworkTables.
 *
 * <p>Protocol (all keys relative to the {@code /LLM} table):
 *
 * <ul>
 *   <li>{@code manifest} (string) – JSON describing every registered command and its parameters.
 *       Published once by the robot; the client uses it to build tool definitions.
 *   <li>{@code commands/<name>/params/<param>} – written by the client before triggering.
 *   <li>{@code commands/<name>/run} (boolean) – client sets true to schedule; robot resets to
 *       false once the request has been consumed.
 *   <li>{@code commands/<name>/cancel} (boolean) – client sets true to cancel a running instance.
 *   <li>{@code commands/<name>/status} (string) – {@code idle}, {@code running}, {@code
 *       finished}, {@code interrupted}, {@code aborted} (the robot-side stall check or watchdog
 *       cancelled it) or {@code rejected}.
 *   <li>{@code commands/<name>/completedCount} (integer) – incremented every time a run ends.
 *       The client compares this against the value before triggering to detect completion
 *       without racing on {@code status}.
 *   <li>{@code commands/<name>/lastResult} (string) – human-readable outcome of the last run.
 *   <li>{@code commands/<name>/expected} (string) – JSON object of the end values the current
 *       (or last) run was expected to reach, keyed by state key. Empty object if none declared.
 *   <li>{@code cancelAll} (boolean) – client sets true to cancel every scheduled command.
 *   <li>{@code state/<key>} – telemetry published by subsystems via {@link #publishState}.
 *   <li>{@code state/avoidance_zones} (string) – JSON array of rectangles the robot must not
 *       enter, maintained via {@link #addAvoidanceZone}. Each element has {@code name}, {@code
 *       min_x}, {@code min_y}, {@code max_x} and {@code max_y} in field meters.
 * </ul>
 *
 * <p>Call {@link #periodic()} from {@code Robot.robotPeriodic()}.
 */
public final class LlmCommands {
  private static final String TABLE_NAME = "LLM";
  private static final int MANIFEST_VERSION = 2;
  /** Change in a tracked numeric value that counts as progress for the stall check. */
  private static final double STALL_EPSILON = 1e-3;
  private static final String AVOIDANCE_ZONES_KEY = "avoidance_zones";

  private static final LlmCommands kInstance = new LlmCommands();

  private final NetworkTable m_table;
  private final NetworkTable m_commandsTable;
  private final NetworkTable m_stateTable;
  private final NetworkTableEntry m_manifestEntry;
  private final NetworkTableEntry m_cancelAllEntry;
  private final ObjectMapper m_mapper = new ObjectMapper();

  private final Map<String, Registration> m_registrations = new LinkedHashMap<>();
  private final List<AvoidanceZone> m_avoidanceZones = new ArrayList<>();
  private boolean m_manifestDirty = true;

  private LlmCommands() {
    m_table = NetworkTableInstance.getDefault().getTable(TABLE_NAME);
    m_commandsTable = m_table.getSubTable("commands");
    m_stateTable = m_table.getSubTable("state");
    m_manifestEntry = m_table.getEntry("manifest");
    m_cancelAllEntry = m_table.getEntry("cancelAll");
    m_cancelAllEntry.setBoolean(false);
    publishAvoidanceZones();
  }

  public static LlmCommands getInstance() {
    return kInstance;
  }

  /** Begin registering a command with the given tool name (snake_case is friendliest to LLMs). */
  public static LlmCommandBuilder register(String name) {
    return new LlmCommandBuilder(kInstance, name);
  }

  /** Publish a telemetry value the LLM can inspect with its {@code get_robot_state} tool. */
  public static void publishState(String key, double value) {
    kInstance.m_stateTable.getEntry(key).setDouble(value);
  }

  public static void publishState(String key, boolean value) {
    kInstance.m_stateTable.getEntry(key).setBoolean(value);
  }

  public static void publishState(String key, String value) {
    kInstance.m_stateTable.getEntry(key).setString(value);
  }

  /**
   * Define a rectangular area of the field the robot must not enter. The rectangle is given by its
   * top-left and bottom-right corners in field coordinates (meters); the corners may be supplied
   * in either order. The zone is published under {@code state/avoidance_zones} so the LLM can
   * plan paths around it, and drivetrain commands consult {@link #getAvoidanceZones()} to refuse
   * motions that would cross one.
   *
   * @param name short label for the zone, e.g. {@code "charging_station"}
   * @param topLeftX x coordinate of the top-left corner
   * @param topLeftY y coordinate of the top-left corner
   * @param bottomRightX x coordinate of the bottom-right corner
   * @param bottomRightY y coordinate of the bottom-right corner
   * @return the normalised zone that was added
   */
  public static AvoidanceZone addAvoidanceZone(
      String name, double topLeftX, double topLeftY, double bottomRightX, double bottomRightY) {
    AvoidanceZone zone =
        AvoidanceZone.fromCorners(
            name,
            new Translation2d(topLeftX, topLeftY),
            new Translation2d(bottomRightX, bottomRightY));
    addAvoidanceZone(zone);
    return zone;
  }

  /** Add an already-constructed zone. See {@link #addAvoidanceZone(String, double, double, double, double)}. */
  public static void addAvoidanceZone(AvoidanceZone zone) {
    kInstance.m_avoidanceZones.add(zone);
    kInstance.publishAvoidanceZones();
    System.out.println("[LLM] added avoidance zone " + zone);
  }

  /** Remove every avoidance zone. */
  public static void clearAvoidanceZones() {
    kInstance.m_avoidanceZones.clear();
    kInstance.publishAvoidanceZones();
  }

  /** The current avoidance zones, in the order they were added. Read-only. */
  public static List<AvoidanceZone> getAvoidanceZones() {
    return Collections.unmodifiableList(kInstance.m_avoidanceZones);
  }

  /**
   * The first avoidance zone crossed by the straight segment from {@code start} to {@code end},
   * or {@code null} if the path is clear.
   */
  public static AvoidanceZone findZoneCrossedBy(Translation2d start, Translation2d end) {
    for (AvoidanceZone zone : kInstance.m_avoidanceZones) {
      if (zone.intersectsSegment(start, end)) {
        return zone;
      }
    }
    return null;
  }

  private void publishAvoidanceZones() {
    ArrayNode zones = m_mapper.createArrayNode();
    for (AvoidanceZone zone : m_avoidanceZones) {
      ObjectNode node = zones.addObject();
      node.put("name", zone.name());
      node.put("min_x", zone.minX());
      node.put("min_y", zone.minY());
      node.put("max_x", zone.maxX());
      node.put("max_y", zone.maxY());
    }
    try {
      // Written via the instance table (not publishState) because this also runs from the
      // constructor, before kInstance has been assigned.
      m_stateTable.getEntry(AVOIDANCE_ZONES_KEY).setString(m_mapper.writeValueAsString(zones));
    } catch (JsonProcessingException e) {
      DriverStation.reportError(
          "[LLM] failed to serialize avoidance zones: " + e.getMessage(), false);
    }
  }

  /** Poll for client requests. Call once per loop from {@code robotPeriodic}. */
  public void periodic() {
    if (m_manifestDirty) {
      m_manifestEntry.setString(buildManifest());
      m_manifestDirty = false;
    }

    publishState("robot/enabled", DriverStation.isEnabled());
    publishState("robot/mode", currentMode());

    if (m_cancelAllEntry.getBoolean(false)) {
      m_cancelAllEntry.setBoolean(false);
      CommandScheduler.getInstance().cancelAll();
      DriverStation.reportWarning("[LLM] cancelAll requested", false);
    }

    for (Registration reg : m_registrations.values()) {
      reg.periodic();
    }
  }

  void add(LlmCommandSpec spec) {
    if (m_registrations.containsKey(spec.name())) {
      throw new IllegalArgumentException("LLM command already registered: " + spec.name());
    }
    m_registrations.put(spec.name(), new Registration(spec));
    m_manifestDirty = true;
  }

  private String buildManifest() {
    ObjectNode root = m_mapper.createObjectNode();
    root.put("version", MANIFEST_VERSION);
    root.put("table", "/" + TABLE_NAME);
    ArrayNode commands = root.putArray("commands");
    for (Registration reg : m_registrations.values()) {
      LlmCommandSpec spec = reg.m_spec;
      ObjectNode cmd = commands.addObject();
      cmd.put("name", spec.name());
      cmd.put("description", spec.description());
      cmd.put("timeoutSeconds", spec.timeoutSeconds());
      cmd.put("checkInSeconds", spec.checkInSeconds());
      cmd.put("stallTimeoutSeconds", spec.stallTimeoutSeconds());
      cmd.put("hasWatchdog", spec.watchdog() != null);
      ArrayNode tracked = cmd.putArray("trackedState");
      spec.trackedState().forEach(tracked::add);
      ArrayNode params = cmd.putArray("parameters");
      for (ParamSpec p : spec.params()) {
        ObjectNode pn = params.addObject();
        pn.put("name", p.name());
        pn.put("type", p.type());
        pn.put("description", p.description());
        if (p.min() != null) {
          pn.put("min", p.min());
        }
        if (p.max() != null) {
          pn.put("max", p.max());
        }
        if (p.choices() != null) {
          ArrayNode choices = pn.putArray("choices");
          p.choices().forEach(choices::add);
        }
      }
    }
    try {
      return m_mapper.writeValueAsString(root);
    } catch (JsonProcessingException e) {
      DriverStation.reportError("[LLM] failed to serialize manifest: " + e.getMessage(), false);
      return "{}";
    }
  }

  private static String currentMode() {
    if (DriverStation.isDisabled()) {
      return "disabled";
    } else if (DriverStation.isAutonomousEnabled()) {
      return "autonomous";
    } else if (DriverStation.isTestEnabled()) {
      return "test";
    }
    return "teleop";
  }

  /** Per-command NetworkTables plumbing and lifecycle tracking. */
  private final class Registration {
    private final LlmCommandSpec m_spec;
    private final NetworkTableEntry m_runEntry;
    private final NetworkTableEntry m_cancelEntry;
    private final NetworkTableEntry m_statusEntry;
    private final NetworkTableEntry m_completedCountEntry;
    private final NetworkTableEntry m_lastResultEntry;
    private final NetworkTableEntry m_expectedEntry;
    private final Map<String, NetworkTableEntry> m_paramEntries = new HashMap<>();

    private Command m_active;
    private LlmRun m_run;
    private String m_abortReason;
    private boolean m_timedOut;
    private double m_lastProgressTime;
    private Map<String, Object> m_lastProgressValues;
    private long m_completedCount;

    Registration(LlmCommandSpec spec) {
      m_spec = spec;
      NetworkTable table = m_commandsTable.getSubTable(spec.name());
      m_runEntry = table.getEntry("run");
      m_cancelEntry = table.getEntry("cancel");
      m_statusEntry = table.getEntry("status");
      m_completedCountEntry = table.getEntry("completedCount");
      m_lastResultEntry = table.getEntry("lastResult");
      m_expectedEntry = table.getEntry("expected");

      m_runEntry.setBoolean(false);
      m_cancelEntry.setBoolean(false);
      m_statusEntry.setString("idle");
      m_completedCountEntry.setInteger(0);
      m_lastResultEntry.setString("");
      m_expectedEntry.setString("{}");

      NetworkTable params = table.getSubTable("params");
      for (ParamSpec p : spec.params()) {
        NetworkTableEntry entry = params.getEntry(p.name());
        // Publish a default so the key exists and the client sees the expected type.
        switch (p.type()) {
          case "double" -> entry.setDefaultDouble(0.0);
          case "integer" -> entry.setDefaultInteger(0);
          case "boolean" -> entry.setDefaultBoolean(false);
          default -> entry.setDefaultString("");
        }
        m_paramEntries.put(p.name(), entry);
      }
    }

    void periodic() {
      if (m_cancelEntry.getBoolean(false)) {
        m_cancelEntry.setBoolean(false);
        if (m_active != null) {
          m_active.cancel();
        }
      }

      if (m_runEntry.getBoolean(false)) {
        m_runEntry.setBoolean(false);
        handleRunRequest();
      }

      if (m_active != null) {
        monitor();
      }
    }

    /**
     * Robot-side health checks, run once per loop while the command is active. Aborting cancels
     * the command synchronously, so {@link #finish} has already run when this returns.
     */
    private void monitor() {
      if (m_spec.timeoutSeconds() > 0 && m_run.elapsedSeconds() >= m_spec.timeoutSeconds()) {
        m_timedOut = true;
        m_active.cancel();
        return;
      }
      String reason = checkStall();
      if (reason == null && m_spec.watchdog() != null) {
        try {
          reason = m_spec.watchdog().check(m_run);
        } catch (RuntimeException e) {
          reason = "watchdog threw: " + e.getMessage();
        }
      }
      if (reason != null) {
        m_abortReason = reason;
        DriverStation.reportWarning("[LLM] aborting " + m_spec.name() + ": " + reason, false);
        m_active.cancel();
      }
    }

    /** Returns a reason if no tracked numeric key has moved for the configured stall timeout. */
    private String checkStall() {
      if (m_spec.stallTimeoutSeconds() <= 0 || m_spec.trackedState().isEmpty()) {
        return null;
      }
      Map<String, Object> now = snapshotTrackedState();
      boolean progressed = false;
      for (String key : m_spec.trackedState()) {
        Object before = m_lastProgressValues.get(key);
        Object after = now.get(key);
        if (before instanceof Number b && after instanceof Number a) {
          if (Math.abs(a.doubleValue() - b.doubleValue()) > STALL_EPSILON) {
            progressed = true;
          }
        } else if (before != null && !before.equals(after)) {
          progressed = true;
        }
      }
      double t = Timer.getFPGATimestamp();
      if (progressed) {
        m_lastProgressTime = t;
        m_lastProgressValues = now;
        return null;
      }
      double stalled = t - m_lastProgressTime;
      if (stalled < m_spec.stallTimeoutSeconds()) {
        return null;
      }
      return String.format(
          "no progress on %s for %.1fs (values %s)", m_spec.trackedState(), stalled, now);
    }

    private Map<String, Object> snapshotTrackedState() {
      Map<String, Object> values = new LinkedHashMap<>();
      for (String key : m_spec.trackedState()) {
        values.put(key, m_stateTable.getEntry(key).getValue().getValue());
      }
      return values;
    }

    private void handleRunRequest() {
      if (m_active != null) {
        // Re-triggering while running restarts with the new parameters.
        m_active.cancel();
      }

      LlmParams params = readParams();

      if (DriverStation.isDisabled()) {
        finish("rejected", "robot is disabled; enable it before running commands");
        return;
      }

      Command cmd;
      try {
        cmd = m_spec.factory().apply(params);
      } catch (RuntimeException e) {
        finish("rejected", "command factory threw: " + e.getMessage());
        return;
      }

      Map<String, Object> expected;
      try {
        expected = new LinkedHashMap<>(m_spec.expected().apply(params));
      } catch (RuntimeException e) {
        finish("rejected", "expected-value function threw: " + e.getMessage());
        return;
      }
      m_expectedEntry.setString(toJson(expected));

      // The timeout is enforced in monitor() rather than with withTimeout(): that wraps the
      // command in a race group that *finishes* normally when the timer wins, which would make a
      // timed-out run indistinguishable from a successful one.
      final Command scheduled =
          cmd.finallyDo(
                  interrupted -> {
                    m_active = null;
                    if (!interrupted) {
                      finish("finished", "command completed");
                    } else if (m_abortReason != null) {
                      finish("aborted", m_abortReason);
                    } else if (m_timedOut) {
                      finish(
                          "interrupted",
                          String.format("timed out after %.1fs", m_spec.timeoutSeconds()));
                    } else if (DriverStation.isDisabled()) {
                      finish("interrupted", "robot was disabled while the command was running");
                    } else {
                      finish("interrupted", "command was cancelled");
                    }
                  })
              .withName("LLM:" + m_spec.name());

      Map<String, Object> startState = snapshotTrackedState();
      m_run = new LlmRun(params, expected, startState, m_stateTable);
      m_abortReason = null;
      m_timedOut = false;
      m_lastProgressTime = Timer.getFPGATimestamp();
      m_lastProgressValues = startState;

      m_active = scheduled;
      m_statusEntry.setString("running");
      System.out.println("[LLM] run " + m_spec.name() + " " + params);
      CommandScheduler.getInstance().schedule(scheduled);
    }

    private String toJson(Map<String, Object> values) {
      try {
        return m_mapper.writeValueAsString(values);
      } catch (JsonProcessingException e) {
        DriverStation.reportError("[LLM] failed to serialize expected values: " + e, false);
        return "{}";
      }
    }

    private LlmParams readParams() {
      Map<String, Object> values = new HashMap<>();
      for (ParamSpec p : m_spec.params()) {
        NetworkTableValue v = m_paramEntries.get(p.name()).getValue();
        values.put(p.name(), v.getValue());
      }
      return new LlmParams(values);
    }

    private void finish(String status, String result) {
      m_completedCount++;
      m_statusEntry.setString(status);
      m_lastResultEntry.setString(result);
      m_completedCountEntry.setInteger(m_completedCount);
      System.out.println("[LLM] " + m_spec.name() + " -> " + status + ": " + result);
    }
  }
}
