package org.team1126.robot.util.nav;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import org.team1126.lib.util.Alliance;
import org.team1126.robot.subsystems.Swerve;
import org.team1126.robot.util.autos.AutosFlip;

/**
 * Abstract navigator that assumes the coordinates will be north heading.
 */
public abstract class NorthDefaultNavigator implements Navigator {

    protected static final double DEFAULT_DECEL = 1.8;
    protected static final double SIMULATION_DECEL = 20.0;
    protected static final double DEFAULT_TOL = 0.5;

    protected Swerve swerve;
    protected WaypointHeading direction = WaypointHeading.NORTH;

    protected Waypoint[] waypoints;

    public Command driveWaypoint(WaypointHeading direction, BooleanSupplier left, int index) {
        return driveWaypoint(direction, left, index, Alliance.isBlue());
    }

    public Command driveWaypoint(WaypointHeading direction, BooleanSupplier left, int index, boolean blue) {
        if (index < waypoints.length) {
            int adj = (direction == WaypointHeading.NORTH) ? index : (waypoints.length - 1) - index;
            Waypoint waypoint = waypoints[adj];
            // if there is a hard stop so that we dokn't drive off the field.
            if (waypoint.limitField) {
                return Commands.none();
            }

            return swerve.apfDrive(
                () -> waypoints[adj].asPose(left, () -> blue),
                () -> waypoints[adj].decel,
                () -> DEFAULT_TOL
            );
        } else {
            return Commands.none();
        }
    }

    public Command heading(WaypointHeading heading) {
        return Commands.runOnce(() -> this.direction = heading)
            .ignoringDisable(true)
            .withName("Navigator.heading()");
    }

    public Command driveWaypoint(AutosFlip flip, int index) {
        return driveWaypoint(direction, () -> flip.shouldFlip(), index, Alliance.isBlue());
    }

    public Command driveWaypoint(AutosFlip flip, int index, boolean blue) {
        return driveWaypoint(direction, () -> flip.shouldFlip(), index, blue);
    }
}
