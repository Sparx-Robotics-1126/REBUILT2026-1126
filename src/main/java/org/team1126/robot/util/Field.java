package org.team1126.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.team1126.lib.math.FieldInfo;
import org.team1126.lib.math.PAPFController.LateralObstacle;
import org.team1126.lib.math.PAPFController.LineObstacle;
import org.team1126.lib.math.PAPFController.LongitudinalObstacle;
import org.team1126.lib.math.PAPFController.Obstacle;
import org.team1126.lib.math.geometry.ExtPose;
import org.team1126.lib.math.geometry.ExtTranslation;
import org.team1126.robot.subsystems.Swerve;

/**
 * Field locations and utilities.
 */
public final class Field {

    /** The center of the field over its length (X direction). Also the CENTER LINE. */
    public static final double X_CENTER = FieldInfo.length() / 2.0;
    /** The center of the field over its width (Y direction). */
    public static final double Y_CENTER = FieldInfo.width() / 2.0;

    /** X location of pivot point, relative to blue, right. */
    public static final double TRENCH_PIVOT_X = getTag(28).getX() - (Swerve.OFFSET * 1.5);
    /** Y location of pivot point, relative to blue, right. */
    public static final double TRENCH_PIVOT_Y = getTag(28).getY();

    /** Inline with the trench (to pass through), so facing opposite of april tag. */
    public static final ExtPose TRENCH_PIVOT = new ExtPose(TRENCH_PIVOT_X, TRENCH_PIVOT_Y, Rotation2d.kZero);

    public static final double ZONE_PIVOT_X = getTag(26).getX() - (Swerve.OFFSET * 2);
    public static final double ZONE_PIVOT_Y = getTag(26).getY() + (Swerve.OFFSET * 2);

    public static final ExtPose ZONE_PIVOT = new ExtPose(ZONE_PIVOT_X, ZONE_PIVOT_Y, Rotation2d.kCW_90deg);

    public static final ExtPose NEUTRAL_ZONE = new ExtPose(
        (FieldInfo.width() / 2.0),
        (FieldInfo.length() / 2.0),
        Rotation2d.k180deg
    );

    public static final ExtPose WAYPOINT_NEAR = new ExtPose(
        new Pose2d((getTag(23).getX() - 1), getTag(23).getY(), Rotation2d.kZero)
    );
    public static final ExtPose WAYPOINT_FAR = new ExtPose(
        new Pose2d((getTag(22).getX() - 1), getTag(22).getY(), Rotation2d.k180deg)
    );

    public static final ExtPose WAYPOINT_GOAL_FAR = new ExtPose(new Pose2d(X_CENTER, Y_CENTER, Rotation2d.kCCW_Pi_2));
    public static final ExtPose WAYPOINT_GOAL_NEAR = new ExtPose(new Pose2d(1.25, Y_CENTER, Rotation2d.kZero));

    public static enum WaypointHeading {
        NORTH,
        SOUTH
    }

    // Temporary
    // DEPOT Drive Coordinates
    public static final double DEPOT_X = Units.inchesToMeters(27.0);
    public static final double DEPOT_Y = Units.inchesToMeters(213.85);
    public static final Rotation2d DEPOT_ROT = Rotation2d.fromDegrees(135);

    /** HUB Center Y Coordinate */
    public static final double FIELD_CENTER_Y = getTag(26).getY();

    /** The width of the hub as indicated on field maps. */
    public static final double HUB_WIDTH = Units.inchesToMeters(47.0);

    /** The X coordinate of the edge of the blue ALLIANCE ZONE. */
    public static final double BLUE_ZONE = getTag(26).getX();
    /** The X coordinate of the edge of the red ALLIANCE ZONE. */
    public static final double RED_ZONE = getTag(10).getX();

    /** AT26 Y point is the center Y for the hub. */
    public static final double HUB_CENTER_Y = getTag(26).getY();

    /** Center X of the hub is half way between AT26 and AT20. */
    public static final double HUB_CENTER_X = getTag(26).getX() + (getTag(20).getX() - getTag(26).getX());

    /** Picking the center of the hub as the hub's location, for the purpose of aiming. */
    public static final ExtTranslation HUB = new ExtTranslation(HUB_CENTER_X, HUB_CENTER_Y);

    ////////////////////////
    // BARRIER DEFINITION //
    ////////////////////////
    /** Magic number (108.5") based on the measurement from the diagrams. */
    public static final double BARRIER_HALF_WIDTH = Units.inchesToMeters(108.5);

    // BARRIER helper coordinates.
    /** LEFT Y coordinate of the barrier. */
    public static final double BARRIER_LEFT_Y = Y_CENTER + BARRIER_HALF_WIDTH;
    /** RIGHT Y coordinate of the barrier. */
    public static final double BARRIER_RIGHT_Y = Y_CENTER - BARRIER_HALF_WIDTH;
    /** NEAR X coordinate of the barrier. */
    public static final double BARRIER_NEAR_X = getTag(26).getX();
    /** FAR X coordinate of the barrier.  */
    public static final double BARRIER_FAR_X = getTag(20).getX();

    /** The near left corner of the HUB, from the perspective of the DRIVER STATION. */
    public static final ExtTranslation BARRIER_NEAR_LEFT_CORNER = new ExtTranslation(BARRIER_NEAR_X, BARRIER_LEFT_Y);
    /** The near right corner of the HUB, from the perspective of the DRIVER STATION. */
    public static final ExtTranslation BARRIER_NEAR_RIGHT_CORNER = new ExtTranslation(BARRIER_NEAR_X, BARRIER_RIGHT_Y);
    /** The far left corner of the HUB, from the perspective of the DRIVER STATION. */
    public static final ExtTranslation BARRIER_FAR_LEFT_CORNER = new ExtTranslation(BARRIER_FAR_X, BARRIER_LEFT_Y);
    /** The far right corner of the HUB, from the perspective of the DRIVER STATION. */
    public static final ExtTranslation BARRIER_FAR_RIGHT_CORNER = new ExtTranslation(BARRIER_FAR_X, BARRIER_RIGHT_Y);

    ////////////////////////
    // TOWER DEFINITION //
    ////////////////////////
    /** Depth of the tower base from the end wall to the edge of the plate. */
    public static final double TOWER_DEPTH = Units.inchesToMeters(45.18);
    /** Half of the width of the tower base. */
    public static final double TOWER_HALF_WIDTH = Units.inchesToMeters(39.0);

    /** Y coordinate of the depot side of the tower. */
    public static final double TOWER_DEPOT_Y = getTag(31).getY() + TOWER_HALF_WIDTH;
    /** Y coordinate of the outpost side of the tower. */
    public static final double TOWER_OUTPOST_Y = getTag(31).getY() - TOWER_HALF_WIDTH;
    /** X coordinate of the driver side wall for the tower base. */
    public static final double TOWER_NEAR_X = getTag(31).getX();
    /** X coordinate of the far edge of the tower base. */
    public static final double TOWER_FAR_X = getTag(31).getX() + TOWER_DEPTH;

    public static final ExtTranslation TOWER_DEPOT_NEAR_CORNER = new ExtTranslation(TOWER_NEAR_X, TOWER_DEPOT_Y);

    public static final ExtTranslation TOWER_DEPOT_FAR_CORNER = new ExtTranslation(TOWER_FAR_X, TOWER_DEPOT_Y);

    public static final ExtTranslation TOWER_OUTPOST_NEAR_CORNER = new ExtTranslation(TOWER_NEAR_X, TOWER_OUTPOST_Y);

    public static final ExtTranslation TOWER_OUTPOST_FAR_CORNER = new ExtTranslation(TOWER_FAR_X, TOWER_OUTPOST_Y);

    // spotless:off

    /** Obstacles for the P-APF. */
    public static final Obstacle[] OBSTACLES = {
        // Field boundary 
        // For the field boundary we will only want to keep 10cm from the wall since
        // we will need to go in the trenches.
        new LongitudinalObstacle(0.0, 1.5, 0.1),
        new LongitudinalObstacle(FieldInfo.length(), 1.5, 0.1),
        new LateralObstacle(0.0, 1.5, 0.1), 
        new LateralObstacle(FieldInfo.width(), 1.5, 0.1),
        
        // Center Line should only be an obstacle during autos.
        new LongitudinalObstacle(X_CENTER, 10, Swerve.OFFSET + 0.5),
        
        // No-Go Zone:
        // We want to repulse from the edges where the hub and the bump are very strongly, but not too far to shoot.
        // However we do not want to repulse too far from the trench sides. From the standpoint of tuning, we could
        // also attempt to be attracted to it.

        // Blue No-Go Zone
        // Near vertext of the barrier
        new LineObstacle(BARRIER_NEAR_LEFT_CORNER.getBlue(), BARRIER_NEAR_RIGHT_CORNER.getBlue(), 10.0, 0.75),
        // Left vertex of the barrier
        new LineObstacle(BARRIER_NEAR_LEFT_CORNER.getBlue(), BARRIER_FAR_LEFT_CORNER.getBlue(), 5.0, 0.05),
        // Far vertex of the barrier 
        new LineObstacle(BARRIER_FAR_LEFT_CORNER.getBlue(), BARRIER_FAR_RIGHT_CORNER.getBlue(), 10.0, 0.75),
        // Right vertex of the barrier
        new LineObstacle(BARRIER_FAR_RIGHT_CORNER.getBlue(), BARRIER_NEAR_RIGHT_CORNER.getBlue(), 5.0, 0.05),
        
        // Red No-Go Zone
        // Near vertext of the barrier
        new LineObstacle(BARRIER_NEAR_LEFT_CORNER.getRed(), BARRIER_NEAR_RIGHT_CORNER.getRed(), 10.0, 0.75),
        // Left vertex of the barrier
        new LineObstacle(BARRIER_NEAR_LEFT_CORNER.getRed(), BARRIER_FAR_LEFT_CORNER.getRed(), 5.0, 0.05),
        // Far vertex of the barrier 
        new LineObstacle(BARRIER_FAR_LEFT_CORNER.getRed(), BARRIER_FAR_RIGHT_CORNER.getRed(), 10.0, 0.75),
        // Right vertex of the barrier
        new LineObstacle(BARRIER_FAR_RIGHT_CORNER.getRed(), BARRIER_NEAR_RIGHT_CORNER.getRed(), 5.0, 0.05),
        
        // Blue Tower
        new LineObstacle(TOWER_DEPOT_NEAR_CORNER.getBlue(), TOWER_DEPOT_FAR_CORNER.getBlue(), 10.0, 0.5),
        new LineObstacle(TOWER_DEPOT_FAR_CORNER.getBlue(), TOWER_OUTPOST_FAR_CORNER.getBlue(), 10.0, 0.5), 
        new LineObstacle(TOWER_OUTPOST_NEAR_CORNER.getBlue(), TOWER_OUTPOST_FAR_CORNER.getBlue(), 10.0, 0.5),
        
        // Red Tower
        new LineObstacle(TOWER_DEPOT_NEAR_CORNER.getRed(), TOWER_DEPOT_FAR_CORNER.getRed(), 10.0, 0.5),
        new LineObstacle(TOWER_DEPOT_FAR_CORNER.getRed(), TOWER_OUTPOST_FAR_CORNER.getRed(), 10.0, 0.5), 
        new LineObstacle(TOWER_OUTPOST_NEAR_CORNER.getRed(), TOWER_OUTPOST_FAR_CORNER.getRed(), 10.0, 0.5),
    };

    // spotless:on

    /**
     * Returns the pose of the specified AprilTag on the field.
     * @param id The ID of the tag.
     * @return A {@link Pose3d} representing the tag's blue origin relative field position.
     */
    private static Pose3d getTag(int id) {
        return FieldInfo.aprilTags().getTagPose(id).get();
    }

    private Field() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
