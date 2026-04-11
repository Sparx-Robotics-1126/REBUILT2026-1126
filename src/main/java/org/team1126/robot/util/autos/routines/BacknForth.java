package org.team1126.robot.util.autos.routines;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.team1126.lib.math.geometry.ExtPose;
import org.team1126.robot.Robot;
import org.team1126.robot.util.autos.AutosFlip;
import org.team1126.robot.util.autos.AutosStart;
import org.team1126.robot.util.autos.BaseAutosRoutine;
import org.team1126.robot.util.nav.Waypoint;

/**
 * Utility class to help with trench navigation. This gives a wrapper around a list of
 * waypoints with their associated decels.
 */
public final class BacknForth extends BaseAutosRoutine {

    public static final String COMMAND_NAME = "BacknForthAutos.action()";
    public static final String DISPLAY_NAME = "Back and Forth";
    public static final String ABBREVIATION = "BNF";

    private static BacknForth instance;

    public static final ExtPose START_AT = new ExtPose(AutosStart.BUMP.getStartingPoint(false, true));
    public static void init(Robot robot) {
        instance = new BacknForth(COMMAND_NAME, DISPLAY_NAME, ABBREVIATION, robot);
    }

    public static BacknForth get() {
        if (instance == null) {
            return null;
        }

        return instance;
    }

    /**
     * Singleton constructor
     */
    private BacknForth(String commandName, String displayName, String abbreviatedName, Robot robot) {
        super(commandName, displayName, abbreviatedName, robot);
        waypoints = Arrays.asList(
            new Waypoint(5.64, 2.611, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(6.329, 2.611, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(6.829, 2.611, Math.toRadians(0.0), getDefaultDecel() * intakeFactor.get()),
            new Waypoint(7.697, 2.611, Math.toRadians(0.0), getDefaultDecel() * intakeFactor.get()),
            new Waypoint(7.697, 3.225, Math.toRadians(90.0), getDefaultDecel() * intakeFactor.get()),
            new Waypoint(6.829, 2.611, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(5.64, 2.611, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(3.299, 2.611, Math.toRadians(45.0), getDefaultDecel()),
            new Waypoint(5.64, 2.611, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(6.329, 2.611, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(6.829, 2.611, Math.toRadians(0.0), getDefaultDecel() * intakeFactor.get()),
            new Waypoint(7.697, 2.611, Math.toRadians(0.0), getDefaultDecel() * intakeFactor.get()),
            new Waypoint(7.697, 1.992, Math.toRadians(270.0), getDefaultDecel() * intakeFactor.get()),
            new Waypoint(6.829, 2.611, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(5.64, 2.611, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(3.299, 2.611, Math.toRadians(45.0), getDefaultDecel())
        ).toArray(new Waypoint[0]);
    }

    public Command action(Supplier<AutosFlip> flip, BooleanSupplier blue) {
        return sequence(
            atStartingPoint(() -> START_AT.get(blue.getAsBoolean(), flip.get().shouldFlip())),
            driveWaypoint(flip, 0, blue)
                .andThen(driveWaypoint(flip, 1, blue))
                .andThen(
                    robot.intake
                        .extendIntake()
                        .withTimeout(1.5)
                        .andThen(robot.intake.moveIntake(false))
                        .withDeadline(driveWaypoint(flip, 2, blue)
                        .andThen(driveWaypoint(flip, 3, blue))
                        .andThen(driveWaypoint(flip, 4, blue)))
                )
                .andThen(driveWaypoint(flip, 5, blue))
                .andThen(driveWaypoint(flip, 6, blue))
                .andThen(robot.intake.moveIntake(false))
                .withDeadline(driveWaypoint(flip, 7, blue)
                .andThen(Commands.waitSeconds(1.0)))
                .andThen(driveWaypoint(flip, 8, blue))
                .andThen(driveWaypoint(flip, 9, blue)),
                driveArchAndShootFuel()
        ).withName(getCommandName());
    }
}
