package org.team1126.robot.util.autos.routines;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.team1126.lib.math.geometry.ExtPose;
import org.team1126.robot.Robot;
import org.team1126.robot.util.autos.AutosFlip;
import org.team1126.robot.util.autos.BaseAutosRoutine;
import org.team1126.robot.util.nav.Waypoint;

public final class HerdIntoSection extends BaseAutosRoutine {

    public static final String COMMAND_NAME = "HerdIntoSection.action()";
    public static final String DISPLAY_NAME = "Herd Into Section";
    public static final String ABBREVIATION = "HIS";

    private static final ExtPose START_AT = new ExtPose(3.539, 0.625, Rotation2d.kZero);
    private static HerdIntoSection instance;

    public static void init(Robot robot) {
        instance = new HerdIntoSection(COMMAND_NAME, DISPLAY_NAME, ABBREVIATION, robot);
    }

    public static HerdIntoSection get() {
        if (instance == null) {
            return null;
        }

        return instance;
    }

    private HerdIntoSection(String commandName, String displayName, String abbreviatedName, Robot robot) {
        super(commandName, displayName, abbreviatedName, robot);
        waypoints = Arrays.asList(
            new Waypoint(5.532, 0.625, Math.toRadians(0.0), 5.0),
            new Waypoint(7.532, 1.51, Math.toRadians(90.0), 5.0),
            new Waypoint(7.532, 1.21, Math.toRadians(100.0), 5.0),
            new Waypoint(7.532, 1.81, Math.toRadians(110.0), 5.0),
            new Waypoint(7.532, 1.51, Math.toRadians(120.0), 5.0),
            new Waypoint(7.532, 2.11, Math.toRadians(130.0), 5.0),
            new Waypoint(7.532, 1.91, Math.toRadians(140.0), 5.0),
            new Waypoint(7.532, 2.51, Math.toRadians(150.0), 5.0),
            new Waypoint(7.532, 2.11, Math.toRadians(160.0), 5.0),
            new Waypoint(7.532, 2.71, Math.toRadians(170.0), 5.0),
            new Waypoint(7.532, 3.594, Math.toRadians(180.0), 5.0),
            new Waypoint(6.532, 3.594, Math.toRadians(270.0), 5.0),
            new Waypoint(6.532, 2.71, Math.toRadians(270.0), getDefaultDecel() * intakeFactor.get()),
            new Waypoint(6.532, 2.11, Math.toRadians(270.0), getDefaultDecel() * intakeFactor.get()),
            new Waypoint(6.532, 2.51, Math.toRadians(270.0), getDefaultDecel() * intakeFactor.get()),
            new Waypoint(6.532, 1.91, Math.toRadians(270.0), getDefaultDecel() * intakeFactor.get()),
            new Waypoint(6.532, 2.11, Math.toRadians(270.0), getDefaultDecel() * intakeFactor.get()),
            new Waypoint(6.532, 1.51, Math.toRadians(270.0), getDefaultDecel() * intakeFactor.get()),
            new Waypoint(6.532, 1.81, Math.toRadians(270.0), getDefaultDecel() * intakeFactor.get()),
            new Waypoint(6.532, 1.21, Math.toRadians(270.0), getDefaultDecel() * intakeFactor.get()),
            new Waypoint(6.532, 1.51, Math.toRadians(270.0), getDefaultDecel()),
            new Waypoint(5.532, 0.625, Math.toRadians(180.0), getDefaultDecel()),
            new Waypoint(3.14, 0.625, Math.toRadians(180.0), getDefaultDecel()),
            new Waypoint(2.849, 0.625, Math.toRadians(90.0), getDefaultDecel()),
            new Waypoint(2.849, 2.315, Math.toRadians(36.0), getDefaultDecel())
        ).toArray(new Waypoint[0]);
    }

    public Command action(Supplier<AutosFlip> flip, BooleanSupplier blue) {
        return sequence(
            atStartingPoint(() -> START_AT.get(blue.getAsBoolean(), flip.get().shouldFlip())),
            robot.intake.extendIntake(false).withTimeout(intakeTimer.getAsDouble()),
            robot.intake
                .moveIntakeMotorCommand(false)
                .withDeadline(
                    driveWaypoint(flip, 0, blue)
                        .andThen(driveWaypoint(flip, 1, blue))
                        .andThen(driveWaypoint(flip, 2, blue))
                        .andThen(driveWaypoint(flip, 3, blue))
                        .andThen(driveWaypoint(flip, 4, blue))
                        .andThen(driveWaypoint(flip, 5, blue))
                        .andThen(driveWaypoint(flip, 6, blue))
                        .andThen(driveWaypoint(flip, 7, blue))
                        .andThen(driveWaypoint(flip, 8, blue))
                        .andThen(driveWaypoint(flip, 9, blue))
                        .andThen(driveWaypoint(flip, 10, blue))
                        .andThen(driveWaypoint(flip, 11, blue))
                        .andThen(driveWaypoint(flip, 12, blue))
                        .andThen(driveWaypoint(flip, 13, blue))
                        .andThen(driveWaypoint(flip, 14, blue))
                        .andThen(driveWaypoint(flip, 15, blue))
                        .andThen(driveWaypoint(flip, 16, blue))
                        .andThen(driveWaypoint(flip, 17, blue))
                        .andThen(driveWaypoint(flip, 18, blue))
                        .andThen(driveWaypoint(flip, 19, blue))
                        .andThen(driveWaypoint(flip, 20, blue))
                        .andThen(driveWaypoint(flip, 21, blue))
                        .andThen(driveWaypoint(flip, 22, blue))
                        .andThen(driveWaypoint(flip, 23, blue))
                        .andThen(driveWaypoint(flip, 24, blue))
                ),
            shootFuel()
        ).withName(getCommandName());
    }
}
