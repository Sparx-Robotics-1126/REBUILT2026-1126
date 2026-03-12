package org.team1126.robot.util.autos.routines;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Arrays;
import org.team1126.lib.math.geometry.ExtPose;
import org.team1126.robot.Robot;
import org.team1126.robot.util.autos.AutosFlip;
import org.team1126.robot.util.autos.AutosStart;
import org.team1126.robot.util.autos.DefaultAutosRoutine;
import org.team1126.robot.util.nav.Waypoint;

/**
 * Utility class to help with trench navigation. This gives a wrapper around a list of
 * waypoints with their associated decels.
 */
public final class IntakeCenter extends DefaultAutosRoutine {

    public static final String COMMAND_NAME = "IntakeCenterAutos.action()";
    public static final String DISPLAY_NAME = "High Across Intake";
    public static final String ABBREVIATION = "HAI";

    private static IntakeCenter instance;

    public static void init(Robot robot) {
        instance = new IntakeCenter(COMMAND_NAME, DISPLAY_NAME, ABBREVIATION, robot);
    }

    public static IntakeCenter get() {
        if (instance == null) {
            return null;
        }

        return instance;
    }

    /**
     * Singleton constructor
     */
    private IntakeCenter(String commandName, String displayName, String abbreviatedName, Robot robot) {
        super(commandName, displayName, abbreviatedName, robot);
        waypoints = Arrays.asList(
            new Waypoint(2.975, 0.603, Math.toRadians(0.0), Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(5.877, 0.603, Math.toRadians(0.0), Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(
                6.191,
                2.248,
                Math.toRadians(0.0),
                Math.toRadians(0.0),
                getDefaultDecel() * intakeFactor.get()
            ),
            new Waypoint(
                7.532,
                1.2,
                Math.toRadians(90.0),
                Math.toRadians(270.0),
                getDefaultDecel() * intakeFactor.get()
            ),
            new Waypoint(
                7.532,
                5.764,
                Math.toRadians(90.0),
                Math.toRadians(270.0),
                getDefaultDecel() * intakeFactor.get()
            ),
            new Waypoint(
                7.532,
                5.764,
                Math.toRadians(90.0),
                Math.toRadians(270.0),
                getDefaultDecel() * intakeFactor.get()
            ),
            new Waypoint(
                6.815,
                7.458,
                Math.toRadians(178.91),
                Math.toRadians(-178.91),
                getDefaultDecel(),
                limitedField.get()
            )
        ).toArray(new Waypoint[0]);
    }

    public Command action(AutosStart startAt, AutosFlip flip, boolean blue) {
        return sequence(
            atStartingPoint(startAt.getStartingPoint(blue)),
            driveArchAndShootFuelStart(),
            atPoint(new ExtPose(2.287, 4.037, Rotation2d.kZero).get(blue, flip.shouldFlip())),
            driveWaypoint(flip, 0, blue)
                .andThen(driveWaypoint(flip, 1, blue))
                .andThen(driveWaypoint(flip, 2, blue))
                .andThen(
                    parallel(
                        robot.intake
                            .extendIntake(false)
                            .withTimeout(1.5)
                            .andThen(robot.intake.moveIntakeMotorCommand(false)),
                        Commands.waitSeconds(2.0)
                            .andThen(driveWaypoint(flip, 3, blue))
                            .andThen(driveWaypoint(flip, 4, blue))
                            .andThen(driveWaypoint(flip, 5, blue))
                            .andThen(driveWaypoint(flip, 6, blue))
                            .andThen(driveWaypoint(flip, 7, blue))
                    )
                )
        ).withName(getCommandName());
    }
}
