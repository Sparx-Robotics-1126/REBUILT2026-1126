package org.team1126.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.function.Supplier;
import org.team1126.lib.math.geometry.ExtPose;

public class WaypointSupplier implements Supplier<Pose2d> {

    private List<ExtPose> waypoints;

    public WaypointSupplier(List<ExtPose> waypoints) {
        this.waypoints = waypoints;
    }

    @Override
    public Pose2d get() {
        if (waypoints != null && waypoints.size() > 0) {
            var waypoint = waypoints.remove(0);
            return waypoint.get();
        } else {
            return null;
        }
    }
}
