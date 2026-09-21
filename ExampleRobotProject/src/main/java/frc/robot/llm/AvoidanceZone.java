// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.llm;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * An axis-aligned rectangular region of the field the robot must not enter.
 *
 * <p>Zones are defined by two opposite corners in field coordinates (meters, WPILib field frame:
 * +x along the field length, +y along its width). The corners are normalised on construction,
 * so they may be given in either order.
 *
 * @param name short label shown to the LLM, e.g. {@code "charging_station"}
 * @param minX smallest x coordinate covered by the zone
 * @param minY smallest y coordinate covered by the zone
 * @param maxX largest x coordinate covered by the zone
 * @param maxY largest y coordinate covered by the zone
 */
public record AvoidanceZone(String name, double minX, double minY, double maxX, double maxY) {
  private static final double kEpsilon = 1e-9;

  /** Build a zone from any two opposite corners (field coordinates, meters). */
  public static AvoidanceZone fromCorners(String name, Translation2d a, Translation2d b) {
    return new AvoidanceZone(
        name,
        Math.min(a.getX(), b.getX()),
        Math.min(a.getY(), b.getY()),
        Math.max(a.getX(), b.getX()),
        Math.max(a.getY(), b.getY()));
  }

  /** Whether the point lies inside (or on the edge of) the zone. */
  public boolean contains(Translation2d point) {
    return point.getX() >= minX
        && point.getX() <= maxX
        && point.getY() >= minY
        && point.getY() <= maxY;
  }

  /**
   * Whether the straight segment from {@code start} to {@code end} passes through the zone. Uses
   * the slab method: clip the segment's parameter range against the x and y extents of the box.
   */
  public boolean intersectsSegment(Translation2d start, Translation2d end) {
    double tMin = 0.0;
    double tMax = 1.0;

    double dx = end.getX() - start.getX();
    if (Math.abs(dx) < kEpsilon) {
      if (start.getX() < minX || start.getX() > maxX) {
        return false;
      }
    } else {
      double t1 = (minX - start.getX()) / dx;
      double t2 = (maxX - start.getX()) / dx;
      tMin = Math.max(tMin, Math.min(t1, t2));
      tMax = Math.min(tMax, Math.max(t1, t2));
    }

    double dy = end.getY() - start.getY();
    if (Math.abs(dy) < kEpsilon) {
      if (start.getY() < minY || start.getY() > maxY) {
        return false;
      }
    } else {
      double t1 = (minY - start.getY()) / dy;
      double t2 = (maxY - start.getY()) / dy;
      tMin = Math.max(tMin, Math.min(t1, t2));
      tMax = Math.min(tMax, Math.max(t1, t2));
    }

    return tMin <= tMax;
  }
}
