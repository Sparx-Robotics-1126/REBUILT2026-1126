package org.team1126.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.team1126.lib.math.FieldInfo;
import org.team1126.lib.math.PAPFController.LateralObstacle;
import org.team1126.lib.math.PAPFController.LineObstacle;
import org.team1126.lib.math.PAPFController.LongitudinalObstacle;
import org.team1126.lib.math.PAPFController.Obstacle;
import org.team1126.lib.math.geometry.ExtTranslation;

/**
 * Field locations and utilities.
 */
public final class Field {

    public static final double ATTRACTIVE_LINE_HALF_LENGTH = 1.5; // 1500 cm

    /** The center of the field over its length (X direction). Also the CENTER LINE. */
    public static final double X_CENTER = FieldInfo.length() / 2.0;
    /** The center of the field over its width (Y direction). */
    public static final double Y_CENTER = FieldInfo.width() / 2.0;

    /** The X coordinate of the edge of the blue ALLIANCE ZONE. */
    public static final double BLUE_ZONE = getTag(26).getX();
    /** The X coordinate of the edge of the red ALLIANCE ZONE. */
    public static final double RED_ZONE = getTag(10).getX();

    // DEPOT Drive Coordinates
    public static final double DEPOT_X = Units.inchesToMeters(27.0);
    public static final double DEPOT_Y = Units.inchesToMeters(213.85);
    public static final Rotation2d DEPOT_ROT = Rotation2d.fromDegrees(135);

    // TRENCH - RIGHT - Attractive Path
    public static final double TRENCH_RIGHT_NEAR_CENTER_BLUE_X = getTag(28).getX();
    public static final double TRENCH_RIGHT_CENTER_BLUE_Y = getTag(28).getY();
    public static final double TRENCH_RIGHT_NEAR_POINT_BLUE_X =
        TRENCH_RIGHT_NEAR_CENTER_BLUE_X - ATTRACTIVE_LINE_HALF_LENGTH;
    public static final double TRENCH_RIGHT_FAR_POINT_BLUE_X =
        TRENCH_RIGHT_NEAR_CENTER_BLUE_X + ATTRACTIVE_LINE_HALF_LENGTH;

    // HUB location helpers
    private static final double HUB_WIDTH = Units.inchesToMeters(47.0);
    private static final double HUB_HALF_WIDTH = HUB_WIDTH / 2.0;
    private static final double HUB_NEAR = BLUE_ZONE;
    private static final double HUB_FAR = HUB_NEAR + HUB_WIDTH;

    /** The center of the HUB. */
    public static final ExtTranslation HUB = new ExtTranslation(HUB_NEAR + HUB_HALF_WIDTH, Y_CENTER);

    // LADDER location helpers
    public static final double BLUE_ZONE_LADDER = getTag(31).getX();

    private static final double LADDER_WIDTH = Units.inchesToMeters(47.0);
    private static final double LADDER_HALF_WIDTH = LADDER_WIDTH / 2.0;
    private static final double LADDER_DRIVER_STATION = BLUE_ZONE_LADDER;
    private static final double LADDER_CLIMBING_RAILS = LADDER_DRIVER_STATION + Units.inchesToMeters(41.56);

    /** TODO: Comment me */
    public static final ExtTranslation LADDER_CLIMBING_RAILS_OUTPOST_SIDE = new ExtTranslation(
        LADDER_CLIMBING_RAILS,
        LADDER_CLIMBING_RAILS - LADDER_HALF_WIDTH
    );

    public static final ExtTranslation LADDER_CLIMBING_RAILS_DEPOT_SIDE = new ExtTranslation(
        LADDER_CLIMBING_RAILS,
        LADDER_CLIMBING_RAILS + LADDER_HALF_WIDTH
    );

    public static final ExtTranslation LADDER_DRIVER_STATION_OUTPOST_SIDE = new ExtTranslation(
        LADDER_DRIVER_STATION,
        LADDER_CLIMBING_RAILS - LADDER_HALF_WIDTH
    );

    public static final ExtTranslation LADDER_DRIVER_STATION_DEPOT_SIDE = new ExtTranslation(
        LADDER_DRIVER_STATION,
        LADDER_CLIMBING_RAILS + LADDER_HALF_WIDTH
    );

    /**
     * Start of the attractive trench line on the right side coming from the BLUE zone.
     */
    public static final ExtTranslation TRENCH_ATTRACTIVE_PATH_RIGHT_START = new ExtTranslation(
        TRENCH_RIGHT_NEAR_POINT_BLUE_X,
        TRENCH_RIGHT_CENTER_BLUE_Y
    );

    /**
     * Start of the attractive trench line on the right side coming from the BLUE zone.
     */
    public static final ExtTranslation TRENCH_ATTRACTIVE_PATH_RIGHT_END = new ExtTranslation(
        TRENCH_RIGHT_FAR_POINT_BLUE_X,
        TRENCH_RIGHT_CENTER_BLUE_Y
    );

    /** The near left corner of the HUB, from the perspective of the DRIVER STATION. */
    public static final ExtTranslation HUB_NEAR_LEFT_CORNER = new ExtTranslation(HUB_NEAR, Y_CENTER + HUB_HALF_WIDTH);
    /** The near right corner of the HUB, from the perspective of the DRIVER STATION. */
    public static final ExtTranslation HUB_NEAR_RIGHT_CORNER = new ExtTranslation(HUB_NEAR, Y_CENTER - HUB_HALF_WIDTH);
    /** The far left corner of the HUB, from the perspective of the DRIVER STATION. */
    public static final ExtTranslation HUB_FAR_LEFT_CORNER = new ExtTranslation(HUB_FAR, Y_CENTER + HUB_HALF_WIDTH);
    /** The far right corner of the HUB, from the perspective of the DRIVER STATION. */
    public static final ExtTranslation HUB_FAR_RIGHT_CORNER = new ExtTranslation(HUB_FAR, Y_CENTER - HUB_HALF_WIDTH);

    // TRENCH location helpers
    private static final double TRENCH_DEPTH = Units.inchesToMeters(47.0);
    private static final double TRENCH_BASE_WIDTH = Units.inchesToMeters(12.0);
    private static final double TRENCH_OFFSET = Units.inchesToMeters(96.5);
    private static final double TRENCH_NEAR = BLUE_ZONE;
    private static final double TRENCH_FAR = TRENCH_NEAR + TRENCH_DEPTH;

    // spotless:off

    /** The near opening side corner of the left TRENCH base, from the perspective of the DRIVER STATION. */
    private static final ExtTranslation LEFT_TRENCH_BASE_NEAR_OPENING_CORNER = new ExtTranslation(TRENCH_NEAR, Y_CENTER + TRENCH_OFFSET + TRENCH_BASE_WIDTH);
    /** The far opening side corner of the left TRENCH base, from the perspective of the DRIVER STATION. */
    private static final ExtTranslation LEFT_TRENCH_BASE_FAR_OPENING_CORNER = new ExtTranslation(TRENCH_FAR, Y_CENTER + TRENCH_OFFSET + TRENCH_BASE_WIDTH);
    /** The near field boundary corner of the left TRENCH, from the perspective of the DRIVER STATION. */

    /** The near opening side corner of the right TRENCH base, from the perspective of the DRIVER STATION. */
    private static final ExtTranslation RIGHT_TRENCH_BASE_NEAR_OPENING_CORNER = new ExtTranslation(TRENCH_NEAR, Y_CENTER - TRENCH_OFFSET - TRENCH_BASE_WIDTH);
    /** The far opening side corner of the right TRENCH base, from the perspective of the DRIVER STATION. */
    private static final ExtTranslation RIGHT_TRENCH_BASE_FAR_OPENING_CORNER = new ExtTranslation(TRENCH_FAR, Y_CENTER - TRENCH_OFFSET - TRENCH_BASE_WIDTH);

    /** Obstacles for the P-APF. */
    public static final Obstacle[] OBSTACLES = {
        // Field boundary 
        // For the field boundary we will only want to keep 10cm from the wall since
        // we will need to go in the trenches.
        new LongitudinalObstacle(0.0, 2.0, 0.1),
        new LongitudinalObstacle(FieldInfo.length(), 2.0, 0.1),
        new LateralObstacle(0.0, 2.0, 0.1), 
        new LateralObstacle(FieldInfo.width(), 2.0, 0.1),
        
        // No-Go Zone:
        // We want to repulse from the edges where the hub and the bump are very strongly, but not too far to shoot.
        // However we do not want to repulse too far from the trench sides. From the standpoint of tuning, we could
        // also attempt to be attracted to it.

        // Blue No-Go Zone
        new LineObstacle(LEFT_TRENCH_BASE_NEAR_OPENING_CORNER.getBlue(), RIGHT_TRENCH_BASE_NEAR_OPENING_CORNER.getBlue(), 10.0, 2.0),
        new LineObstacle(LEFT_TRENCH_BASE_NEAR_OPENING_CORNER.getBlue(), LEFT_TRENCH_BASE_FAR_OPENING_CORNER.getBlue(), 1.0, 0.05),
        new LineObstacle(LEFT_TRENCH_BASE_FAR_OPENING_CORNER.getBlue(), RIGHT_TRENCH_BASE_FAR_OPENING_CORNER.getBlue(), 10.0, 1.0),
        new LineObstacle(RIGHT_TRENCH_BASE_NEAR_OPENING_CORNER.getBlue(), RIGHT_TRENCH_BASE_FAR_OPENING_CORNER.getBlue(), 1.0, 0.05),

        // Red No-Go Zone
        new LineObstacle(LEFT_TRENCH_BASE_NEAR_OPENING_CORNER.getRed(), RIGHT_TRENCH_BASE_NEAR_OPENING_CORNER.getRed(), 10.0, 2.0),
        new LineObstacle(LEFT_TRENCH_BASE_NEAR_OPENING_CORNER.getRed(), LEFT_TRENCH_BASE_FAR_OPENING_CORNER.getRed(), 1.0, 0.25),
        new LineObstacle(LEFT_TRENCH_BASE_FAR_OPENING_CORNER.getRed(), RIGHT_TRENCH_BASE_FAR_OPENING_CORNER.getRed(), 10.0, 1.0),
        new LineObstacle(RIGHT_TRENCH_BASE_NEAR_OPENING_CORNER.getRed(), RIGHT_TRENCH_BASE_FAR_OPENING_CORNER.getRed(), 1.0, 0.25),
        
        // Temporarily highly repulsed from the ladder. That will need to change if/when we can climb
        // Blue Ladder Zone
        new LineObstacle(LADDER_CLIMBING_RAILS_OUTPOST_SIDE.getBlue(), LADDER_DRIVER_STATION_OUTPOST_SIDE.getBlue(), 5.0, 1.0),
        new LineObstacle(LADDER_DRIVER_STATION_OUTPOST_SIDE.getBlue(), LADDER_DRIVER_STATION_DEPOT_SIDE.getBlue(), 5.0, 1.0),
        new LineObstacle(LADDER_DRIVER_STATION_DEPOT_SIDE.getBlue(), LADDER_CLIMBING_RAILS_DEPOT_SIDE.getBlue(), 5.0, 1.0),
        new LineObstacle(LADDER_CLIMBING_RAILS_DEPOT_SIDE.getBlue(), LADDER_CLIMBING_RAILS_OUTPOST_SIDE.getBlue(), 1.0, 0.25),
        
        // Red Ladder Zone
        new LineObstacle(LADDER_CLIMBING_RAILS_OUTPOST_SIDE.getRed(), LADDER_DRIVER_STATION_OUTPOST_SIDE.getRed(), 5.0, 1.0),
        new LineObstacle(LADDER_DRIVER_STATION_OUTPOST_SIDE.getRed(), LADDER_DRIVER_STATION_DEPOT_SIDE.getRed(), 5.0, 1.0),
        new LineObstacle(LADDER_DRIVER_STATION_DEPOT_SIDE.getRed(), LADDER_CLIMBING_RAILS_DEPOT_SIDE.getRed(), 5.0, 1.0),
        new LineObstacle(LADDER_CLIMBING_RAILS_DEPOT_SIDE.getRed(), LADDER_CLIMBING_RAILS_OUTPOST_SIDE.getRed(), 1.0, 0.25),
        
        // Attractive Trench Blue Right
        new LineObstacle(TRENCH_ATTRACTIVE_PATH_RIGHT_START.getBlue(), TRENCH_ATTRACTIVE_PATH_RIGHT_END.getBlue(), -100.0, 1.268),
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
