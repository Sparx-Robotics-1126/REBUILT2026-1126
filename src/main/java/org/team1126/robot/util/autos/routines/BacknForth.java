package org.team1126.robot.util.autos.routines;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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
    
    public static final ExtPose START_AT = new ExtPose(AutosStart.BUMP.getStartingPoint(true, false));
    public static final Waypoint bumpPivot = new Waypoint(5.712, 2.5, Math.toRadians(0.0),getDefaultDecel());
    public static final Waypoint centerLinePivot = new Waypoint(7.692, 0.685, Math.toRadians(90.0), getDefaultDecel());
    public static final Waypoint middlePivot = new Waypoint(7.692, 3.46, Math.toRadians(90.0), getDefaultDecel() * intakeFactor.get());
    public static final Waypoint shootingPoint = new Waypoint(2.74, 2.8, Math.toRadians(45.0), getDefaultDecel() * intakeFactor.get());

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
        waypoints = new Waypoint[0];
    }

    public Command action(Supplier<AutosFlip> flip, BooleanSupplier blue) {
        return sequence(
            atStartingPoint(() -> START_AT.get(blue.getAsBoolean(), flip.get().shouldFlip())),
            routine(flip, blue).andThen(routine(flip, blue))
        ).withName(getCommandName());
    }
    
    private Command routine(Supplier<AutosFlip> flip, BooleanSupplier blue) {
        return driveWaypoint(flip, 0, blue)
            .andThen(driveWaypoint(flip, bumpPivot, blue))
            .andThen(driveWaypoint(flip, centerLinePivot, blue))
            .andThen(robot.intake.extendIntake().withDeadline(Commands.waitSeconds(1.25)))
            .andThen(robot.intake.moveIntake(false).withDeadline(driveWaypoint(flip, middlePivot, blue).andThen(driveWaypoint(flip, bumpPivot, blue))))
            .andThen(driveWaypoint(flip, shootingPoint, blue))
            .andThen(driveArchAndShootFuel().withDeadline(Commands.waitSeconds(5.0)));
    }
}
