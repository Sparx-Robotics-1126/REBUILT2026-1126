package org.team1126.robot.util.autos.routines;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.team1126.robot.Robot;
import org.team1126.robot.util.autos.AutosFlip;
import org.team1126.robot.util.autos.AutosStart;
import org.team1126.robot.util.autos.BaseAutosRoutine;
import org.team1126.robot.util.nav.Waypoint;

/**
 * Utility class to help with trench navigation. This gives a wrapper around a list of
 * waypoints with their associated decels.
 */
public final class SweepCenter extends BaseAutosRoutine {

    public static final String COMMAND_NAME = "SweepCenterAutos.action()";
    public static final String DISPLAY_NAME = "Sweep High, Intake None";
    public static final String ABBREVIATION = "SHIN";

    private static SweepCenter instance;

    public static void init(Robot robot) {
        instance = new SweepCenter(COMMAND_NAME, DISPLAY_NAME, ABBREVIATION, robot);
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
    private SweepCenter(String commandName, String displayName, String abbreviatedName, Robot robot) {
        super(commandName, displayName, abbreviatedName, robot);
        waypoints = Arrays.asList(
            new Waypoint(2.975, 0.603, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(5.877, 0.603, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(7.532, 1.2, Math.toRadians(90.0), getDefaultDecel()),
            new Waypoint(7.532, 5.764, Math.toRadians(90.0), getDefaultDecel()),
            new Waypoint(7.532, 5.764, Math.toRadians(90.0), getDefaultDecel()),
            new Waypoint(6.815, 7.458, Math.toRadians(178.91), getDefaultDecel()),
            new Waypoint(2.70, 7.458, Math.toRadians(178.91), getDefaultDecel(), limitedField.get())
        ).toArray(new Waypoint[0]);
    }

    public Command action(Supplier<AutosStart> startAt, Supplier<AutosFlip> flip, BooleanSupplier blue) {
        return sequence(
            atStartingPoint(() -> startAt.get().getStartingPoint(blue.getAsBoolean())),
            driveArchAndShootFuelStart(),
            driveWaypoint(flip, 0, blue)
                .andThen(driveWaypoint(flip, 1, blue))
                .andThen(driveWaypoint(flip, 2, blue))
                .andThen(driveWaypoint(flip, 3, blue))
                .andThen(driveWaypoint(flip, 4, blue))
                .andThen(driveWaypoint(flip, 5, blue))
                .andThen(driveWaypoint(flip, 6, blue))
        ).withName(getCommandName());
    }
}
