package org.team1126.robot.util.nav;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team1126.robot.subsystems.Swerve;

/**
 * Abstract navigator that assumes the coordinates will be north heading.
 */
public abstract class NorthDefaultNavigator implements Navigator {

    protected static final double DEFAULT_DECEL = 0.3;
    protected static final double SIMULATION_DECEL = 20.0;
    protected static final double DEFAULT_TOL = 0.1;

    protected Swerve swerve;
    protected WaypointHeading direction = WaypointHeading.NORTH;

    protected Waypoint[] waypoints;

    public Command driveWaypoint(WaypointHeading direction, int index) {
        if (index < waypoints.length) {
            int adj = (direction == WaypointHeading.NORTH) ? index : (waypoints.length - 1) - index;
            return swerve.apfDrive(waypoints[adj].asPose(), () -> waypoints[adj].decel, () -> DEFAULT_TOL);
        } else {
            return Commands.none();
        }
    }

    public Command heading(WaypointHeading heading) {
        return Commands.runOnce(() -> this.direction = heading)
            .ignoringDisable(true)
            .withName("Navigator.heading()");
    }
}
