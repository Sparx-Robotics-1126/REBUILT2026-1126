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

    // Temporary
    // DEPOT Drive Coordinates
    public static final double DEPOT_X = Units.inchesToMeters(27.0);
    public static final double DEPOT_Y = Units.inchesToMeters(213.85);
    public static final Rotation2d DEPOT_ROT = Rotation2d.fromDegrees(135);

    /** The center of the field over its length (X direction). Also the CENTER LINE. */
    public static final double X_CENTER = FieldInfo.length() / 2.0;
    /** The center of the field over its width (Y direction). */
    public static final double Y_CENTER = FieldInfo.width() / 2.0;

    /** HUB Center Y Coordinate */
    public static final double HUB_CENTER_Y = getTag(26).getY();

    /** Magic number (108.5") based on the measurement from the diagrams. */
    public static final double HUB_HALF_WIDTH = Units.inchesToMeters(108.5);

    // BARRIER helper coordinates.
    /** LEFT Y coordinate of the barrier. */
    public static final double BARRIER_LEFT_Y = HUB_CENTER_Y + HUB_HALF_WIDTH;
    /** RIGHT Y coordinate of the barrier. */
    public static final double BARRIER_RIGHT_Y = HUB_CENTER_Y - HUB_HALF_WIDTH;
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
