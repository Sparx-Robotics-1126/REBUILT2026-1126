package org.team1126.robot.util.autos.routines;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Arrays;
import org.team1126.lib.math.geometry.ExtPose;
import org.team1126.robot.Robot;
import org.team1126.robot.subsystems.Swerve;
import org.team1126.robot.util.Field;
import org.team1126.robot.util.autos.AutosFlip;
import org.team1126.robot.util.autos.DefaultAutosRoutine;
import org.team1126.robot.util.nav.Waypoint;

/**
 * Starting point: Just south of blue line in line with trench
 * Step 1: Parallel:
 *            * Activate intake
 *            * Drive to turning point
 * Step 2: Pick up balls to center line
 * Step 3: Turn toward hub, come back through same side
 * Step 4: Shoot
 */
public final class GrabAndShoot extends DefaultAutosRoutine {

    public static final String COMMAND_NAME = "GrabAndShoot.action()";
    public static final String DISPLAY_NAME = "Grab And Shoot";
    public static final String ABBREVIATION = "GAS";

    private static GrabAndShoot instance;

    public static void init(Robot robot) {
        instance = new GrabAndShoot(COMMAND_NAME, DISPLAY_NAME, ABBREVIATION, robot);
    }

    public static GrabAndShoot get() {
        if (instance == null) {
            return null;
        }

        return instance;
    }

    /**
     * Singleton constructor
     */
    private GrabAndShoot(String commandName, String displayName, String abbreviatedName, Robot robot) {
        super(commandName, displayName, abbreviatedName, robot);
        waypoints = Arrays.asList(
            new Waypoint(
                (Field.CENTER_X - Swerve.OFFSET - 0.50),
                1.2,
                Math.toRadians(90.0),
                getDefaultDecel() * intakeFactor.get()
            ),
            new Waypoint(
                (Field.CENTER_X - Swerve.OFFSET - 0.50),
                Field.CENTER_Y,
                Math.toRadians(90.0),
                getDefaultDecel() * intakeFactor.get()
            ),
            new Waypoint(
                (Field.CENTER_X - Swerve.OFFSET - 0.50),
                Field.CENTER_Y,
                Math.toRadians(180.0),
                getDefaultDecel() * intakeFactor.get()
            ),
            new Waypoint(
                (Field.CENTER_X - Swerve.OFFSET - 1.50),
                Field.CENTER_Y,
                Math.toRadians(180.0),
                getDefaultDecel() * intakeFactor.get()
            ),
            new Waypoint(
                (Field.CENTER_X - Swerve.OFFSET - 1.50),
                Field.CENTER_Y,
                Math.toRadians(0.0),
                getDefaultDecel()
            ),
            new Waypoint(5.005, 0.625, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(3.14, 0.625, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(2.849, 0.625, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(2.849, 2.315, Math.toRadians(36.0), getDefaultDecel())
        ).toArray(new Waypoint[0]);
    }

    public Command action(AutosFlip flip) {
        return sequence(
            atStartingPoint(new ExtPose(3.539, 0.625, Rotation2d.kZero).get(flip.shouldFlip())),
            robot.intake
                .extendIntake(false)
                .withTimeout(intakeTimer.getAsDouble())
                .andThen(
                    robot.intake
                        .moveIntakeMotorCommand(false)
                        .withDeadline(driveWaypoint(flip, 0))
                        .andThen(driveWaypoint(flip, 1))
                        .andThen(driveWaypoint(flip, 2))
                        .andThen(driveWaypoint(flip, 3))
                ),
            driveWaypoint(flip, 4)
                .andThen(driveWaypoint(flip, 5))
                .andThen(
                    driveWaypoint(flip, 6)
                        .andThen(driveWaypoint(flip, 7))
                        .andThen(driveWaypoint(flip, 8))
                        .andThen(driveArchAndShootFuel())
                )
        ).withName(getCommandName());
    }
}
