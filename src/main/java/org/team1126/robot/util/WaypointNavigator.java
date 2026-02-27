package org.team1126.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.HashMap;
import java.util.Map;
import org.team1126.lib.math.geometry.ExtPose;
import org.team1126.lib.math.geometry.ExtTranslation;

/**
 * Defining pre-calculated field waypoints. All points in this utility class are relative to the blue alliance and can be
 * transposed to the red alliance with the implemented classes.
 */
public class WaypointNavigator {

    private static final double EQUATOR = 182.115;
    // 5 div 8, but actually 4 div 8 since idx operates from 0-4
    private static final double WAYPOINT_COEFFICIENT = 0.5;

    private static final Map<Integer, ExtTranslation> waypointMap = new HashMap<Integer, ExtTranslation>();

    // precalculated waypoints
    static {
        waypointMap.put(0, new ExtTranslation(3.026, 0.667));
        waypointMap.put(1, new ExtTranslation(2.584, 2.167));
        waypointMap.put(2, new ExtTranslation(2.261, 4.075)); // this one is done
        waypointMap.put(3, new ExtTranslation(2.584, 5.190));
        waypointMap.put(4, new ExtTranslation(3.026, 7.502));
    }

    public static enum WaypointHeading {
        NORTH(0),
        SOUTH(1);

        private final int val;

        private WaypointHeading(int val) {
            this.val = val;
        }

        public int getVal() {
            return val;
        }
    }

    public static enum Nudge {
        LEFT(0),
        RIGHT(1),
        CENTER(2);

        private final int val;

        private Nudge(int val) {
            this.val = val;
        }

        public int getVal() {
            return val;
        }
    }

    /**
     * Returns the numbered zone of the field that the robot is in given the
     * pose being passed in. In order for this function to return the correct
     * zone, the current pose must be south of the equator.
     *
     * @param currentPose the current Pose2d of the robot.
     * @return the numeric zone 0-4 that the robot is in or -1 if the robot is
     *         north of the equator.
     */
    public static ExtTranslation waypointForShooting(Pose2d currentPose, Nudge nudge) {
        ExtPose abstractPose = new ExtPose(currentPose);
        WaypointHeading heading = currentWaypointHeading(currentPose);
        WaypointHeading currentSide = heading == WaypointHeading.NORTH ? WaypointHeading.SOUTH : WaypointHeading.NORTH;

        if (currentSide == WaypointHeading.NORTH) {
            return null;
        }

        int currentZone = (int) Math.round(abstractPose.getBlue().getY() * WAYPOINT_COEFFICIENT);
        switch (nudge) {
            case LEFT:
                if (currentZone < 4) {
                    currentZone++;
                }
                break;
            case RIGHT:
                if (currentZone > 0) {
                    currentZone--;
                }
                break;
            default:
                break;
        }

        return waypointMap.get(currentZone);
    }

    /**
     * Returns which hemisphere the supplied current pose is in assuming a lateral line across the center X of the hub as the equator.
     *
     * @param currentPose intended to be the current position of the robot, theoretically could be any given position on the field.
     * @return WaypointHeading of the current hemisphere.
     */
    public static WaypointHeading currentWaypointHeading(Pose2d currentPose) {
        ExtPose abstractPose = new ExtPose(currentPose);
        return abstractPose.getBlue().getX() < EQUATOR ? WaypointHeading.NORTH : WaypointHeading.SOUTH;
    }

    private WaypointNavigator() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
