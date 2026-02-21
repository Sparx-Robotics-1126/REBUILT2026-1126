package org.team1126.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.List;
import org.team1126.lib.math.geometry.ExtPose;
import org.team1126.lib.math.geometry.ExtRotation;
import org.team1126.lib.math.geometry.ExtTranslation;
import org.team1126.lib.util.Alliance;

/**
 * Defining pre-calculated field waypoints. All points in this utility class are relative to the blue alliance and can be
 * transposed to the red alliance with the implemented classes.
 */
public class WaypointNavigator {

    /**
     * Waypoints have been pre-calculated for REBUILT's field.
     *
     * Breakout is intended to be ordered as: WaypointHeading.NORTH, WaypointHeading.SOUTH
     */
    private static final ExtTranslation[][] WAYPOINTS = {
        {
            new ExtTranslation(6.070, 0.667),
            new ExtTranslation(5.684, 2.167),
            new ExtTranslation(Field.CENTER_X, Field.CENTER_Y),
            new ExtTranslation(5.684, 5.190),
            new ExtTranslation(6.070, 7.502)
        },
        {
            new ExtTranslation(3.026, 0.667),
            new ExtTranslation(2.584, 2.167),
            new ExtTranslation(2.305, Field.CENTER_Y),
            new ExtTranslation(2.584, 5.190),
            new ExtTranslation(3.026, 7.502)
        }
    };

    /**
     * Rotations are [N,S][E,W][Waypoint]
     */
    private static final ExtRotation[][][] ROTATIONS = {
        {
            {
                new ExtRotation(0.0),
                new ExtRotation(90.0),
                new ExtRotation(90.0),
                new ExtRotation(90.0),
                new ExtRotation(90.0)
            },
            {
                new ExtRotation(270.0),
                new ExtRotation(270.0),
                new ExtRotation(270.0),
                new ExtRotation(270.0),
                new ExtRotation(0.0)
            }
        },
        {
            {
                new ExtRotation(180.0),
                new ExtRotation(180.0),
                new ExtRotation(180.0),
                new ExtRotation(180.0),
                new ExtRotation(180.0)
            },
            {
                new ExtRotation(180.0),
                new ExtRotation(180.0),
                new ExtRotation(180.0),
                new ExtRotation(180.0),
                new ExtRotation(180.0)
            }
        }
    };

    private static final double EQUATOR = 182.115;
    // 5 div 8, but actually 4 div 8 since idx operates from 0-4
    private static final double WAYPOINT_COEFFICIENT = 0.5;

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

    public static List<ExtPose> trenching(Pose2d currentPose, boolean right) {
        // 0. Make incoming pose relative to blue.
        ExtPose abstractPose = new ExtPose(currentPose);
        // 1. determine if I am north or south of the equator
        // Remember that the heading is the opposite of where we actually are.
        WaypointHeading heading = currentWaypointHeading(currentPose);
        WaypointHeading currentSide = heading == WaypointHeading.NORTH ? WaypointHeading.SOUTH : WaypointHeading.NORTH;

        int startZone = (int) Math.round(abstractPose.getBlue().getY() * WAYPOINT_COEFFICIENT);

        List<ExtPose> path = new ArrayList<>();
        if (right) {
            for (int i = startZone; i >= 0; i--) {
                path.add(
                    new ExtPose(
                        new Pose2d(
                            WAYPOINTS[currentSide.getVal()][i].getBlue(),
                            ROTATIONS[currentSide.getVal()][0][i].getBlue()
                        )
                    )
                );
            }
            for (int i = 0; i < 3; i++) {
                path.add(
                    new ExtPose(
                        new Pose2d(
                            WAYPOINTS[heading.getVal()][i].get(Alliance.isBlue(), false),
                            ROTATIONS[heading.getVal()][0][i].get(Alliance.isBlue(), false)
                        )
                    )
                );
            }
        } else {
            for (int i = startZone; i < WAYPOINTS[heading.getVal()].length; i++) {
                path.add(
                    new ExtPose(
                        new Pose2d(
                            WAYPOINTS[heading.getVal()][i].get(Alliance.isBlue(), false),
                            ROTATIONS[heading.getVal()][0][i].get(Alliance.isBlue(), false)
                        )
                    )
                );
            }
            for (int i = (WAYPOINTS.length - 1); i >= 2; i--) {
                path.add(
                    new ExtPose(
                        new Pose2d(
                            WAYPOINTS[currentSide.getVal()][i].get(Alliance.isBlue(), false),
                            ROTATIONS[currentSide.getVal()][0][i].get(Alliance.isBlue(), false)
                        )
                    )
                );
            }
        }

        return path;
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
