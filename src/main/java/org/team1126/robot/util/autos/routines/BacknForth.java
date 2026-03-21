package org.team1126.robot.util.autos.routines;

import static edu.wpi.first.wpilibj2.command.Commands.*;

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
public final class BacknForth extends BaseAutosRoutine {

    public static final String COMMAND_NAME = "BacknForthAutos.action()";
    public static final String DISPLAY_NAME = "Back and Forth";
    public static final String ABBREVIATION = "BNF";

    private static BacknForth instance;

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
            new Waypoint(7.697, 2.611, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(5.64, 2.611, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(3.299, 2.611, Math.toRadians(45.0), getDefaultDecel()),
            new Waypoint(5.64, 2.611, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(7.697, 2.611, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(5.64, 2.611, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(3.299, 2.611, Math.toRadians(45.0), getDefaultDecel())
        ).toArray(new Waypoint[0]);
    }

    // public Command action(Supplier<AutosStart> startAt, Supplier<AutosFlip> flip, BooleanSupplier blue) {
    //     return sequence(
    //         atStartingPoint(() -> startAt.get().getStartingPoint(blue.getAsBoolean(), flip.get().shouldFlip())),
    //         driveWaypoint(flip, 0, blue),
    //         driveArchAndShootFuelStart(),
    //         atPoint(() ->
    //             new ExtPose(2.287, 4.037, Rotation2d.kZero).get(blue.getAsBoolean(), flip.get().shouldFlip())
    //         ),
    //         driveWaypoint(flip, 1, blue)
    //             .andThen(driveWaypoint(flip, 2, blue))
    //             .andThen(driveWaypoint(flip, 3, blue))
    //             .andThen(
    //                 parallel(
    //                     robot.intake
    //                         .extendIntake(false)
    //                         .withTimeout(1.5)
    //                         .andThen(robot.intake.moveIntakeMotorCommand(false)),
    //                     Commands.waitSeconds(2.0)
    //                         .andThen(driveWaypoint(flip, 4, blue))
    //                         .andThen(driveWaypoint(flip, 5, blue))
    //                         .andThen(driveWaypoint(flip, 6, blue))
    //                         .andThen(driveWaypoint(flip, 7, blue))
    //                         .andThen(driveWaypoint(flip, 8, blue))
    //                 )
    //             )
    //     ).withName(getCommandName());
    // }
    public Command action(Supplier<AutosStart> startAt, Supplier<AutosFlip> flip, BooleanSupplier blue) {
        return sequence(
            atStartingPoint(() -> startAt.get().getStartingPoint(blue.getAsBoolean(), flip.get().shouldFlip())),
            driveWaypoint(flip, 0, blue)
                .andThen(driveWaypoint(flip, 1, blue))
                .andThen(driveWaypoint(flip, 2, blue))
                .andThen(driveWaypoint(flip, 3, blue))
                .andThen(driveWaypoint(flip, 4, blue))
                .andThen(driveWaypoint(flip, 5, blue))
                .andThen(driveWaypoint(flip, 6, blue))
                .andThen(driveWaypoint(flip, 7, blue))
        ).withName(getCommandName());
    }
}
