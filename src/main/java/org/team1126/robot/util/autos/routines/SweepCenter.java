package org.team1126.robot.util.autos.routines;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.Arrays;
import org.team1126.robot.Robot;
import org.team1126.robot.subsystems.Swerve;
import org.team1126.robot.util.Field;
import org.team1126.robot.util.autos.AutosFlip;
import org.team1126.robot.util.autos.AutosStart;
import org.team1126.robot.util.autos.DefaultAutosRoutine;
import org.team1126.robot.util.nav.Waypoint;

/**
 * Utility class to help with trench navigation. This gives a wrapper around a list of
 * waypoints with their associated decels.
 */
public final class SweepCenter extends DefaultAutosRoutine {

    public static final String COMMAND_NAME = "SweepCenterAutos.action()";
    public static final String DISPLAY_NAME = "Sweep center line, no intake";

    private static SweepCenter instance;

    public static void init(Swerve swerve, Robot robot) {
        instance = new SweepCenter(COMMAND_NAME, DISPLAY_NAME, swerve, robot);
    }

    public static SweepCenter get() {
        if (instance == null) {
            return null;
        }

        return instance;
    }

    /**
     * Singleton constructor
     */
    private SweepCenter(String commandName, String displayName, Swerve swerve, Robot robot) {
        super(commandName, displayName, swerve, robot);
        waypoints = Arrays.asList(
            new Waypoint(2.975, 0.603, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(5.877, 0.603, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(Field.CENTER_X - Swerve.OFFSET - 0.25, 1.2, Math.toRadians(270.0), getDefaultDecel()),
            new Waypoint(Field.CENTER_X - Swerve.OFFSET - 0.25, 5.764, Math.toRadians(270.0), getDefaultDecel()),
            new Waypoint(Field.CENTER_X - Swerve.OFFSET - 0.25, 5.764, Math.toRadians(270.0), getDefaultDecel()),
            new Waypoint(6.815, 7.458, Math.toRadians(178.91), getDefaultDecel()),
            new Waypoint(2.70, 7.458, Math.toRadians(178.91), getDefaultDecel(), limitedField.get())
        ).toArray(new Waypoint[0]);
    }

    public Command action(AutosStart startAt, AutosFlip flip) {
        return sequence(
            swerve.resetPose(startAt.getStartingPoint()),
            routines.readyFeederShooter().withTimeout(1.00),
            routines.shootFuelAuto().withTimeout(5.0),
            driveWaypoint(direction, () -> flip.shouldFlip(), 0)
                .andThen(driveWaypoint(direction, () -> flip.shouldFlip(), 1))
                .andThen(driveWaypoint(direction, () -> flip.shouldFlip(), 2))
                .andThen(driveWaypoint(direction, () -> flip.shouldFlip(), 3))
                .andThen(driveWaypoint(direction, () -> flip.shouldFlip(), 4))
                .andThen(driveWaypoint(direction, () -> flip.shouldFlip(), 5))
                .andThen(driveWaypoint(direction, () -> flip.shouldFlip(), 6))
        ).withName(commandName);
    }
}
