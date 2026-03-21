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

public class ShootIntakeShootAtAlliance extends BaseAutosRoutine {

    public static final String COMMAND_NAME = "AllianceShootAutos.action";
    public static final String DISPLAY_NAME = "Shoot, Intake, Shoot at Alliance";
    public static final String ABBREVIATION = "SISAA";

    private static ShootIntakeShootAtAlliance instance;

    public static void init(Robot robot) {
        instance = new ShootIntakeShootAtAlliance(COMMAND_NAME, DISPLAY_NAME, ABBREVIATION, robot);
    }

    public static ShootIntakeShootAtAlliance get() {
        if (instance == null) {
            return null;
        }
        return instance;
    }

    private ShootIntakeShootAtAlliance(String commandName, String displayName, String abbreviatedName, Robot robot) {
        super(commandName, displayName, abbreviatedName, robot);
        waypoints = Arrays.asList(
            new Waypoint(3.028, 2.425, Math.toRadians(40), getDefaultDecel()),
            new Waypoint(2.975, 0.706, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(3.412, 0.706, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(5.412, 0.706, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(7.562, 1.263, Math.toRadians(90), getDefaultDecel()),
            new Waypoint(7.562, 1.628, Math.toRadians(90), 0.4),
            new Waypoint(7.562, 1.750, Math.toRadians(90), 0.4),
            new Waypoint(7.562, 1.900, Math.toRadians(90), 0.4),
            new Waypoint(7.562, 2.150, Math.toRadians(90), 0.4),
            new Waypoint(7.562, 2.452, Math.toRadians(90), 0.4),
            new Waypoint(7.562, 2.650, Math.toRadians(90), 0.4),
            new Waypoint(6.853, 2.602, Math.toRadians(190), getDefaultDecel())
        ).toArray(new Waypoint[0]);
    }

    public Command action(Supplier<AutosStart> startAt, Supplier<AutosFlip> flip, BooleanSupplier blue) {
        return sequence(
            atStartingPoint(() -> startAt.get().getStartingPoint(blue.getAsBoolean(), flip.get().shouldFlip())),
            driveWaypoint(flip, 0, blue),
            driveArchAndShootFuelStart(),
            driveWaypoint(flip, 1, blue)
                .andThen(driveWaypoint(flip, 2, blue))
                .andThen(driveWaypoint(flip, 3, blue))
                .andThen(
                    robot.intake
                        .extendIntake(false)
                        .withTimeout(0.5)
                        .andThen(robot.intake.moveIntakeMotorCommand(false))
                        .withDeadline(driveWaypoint(flip, 4, blue))
                        .andThen(driveWaypoint(flip, 5, blue))
                        .andThen(driveWaypoint(flip, 6, blue))
                        .andThen(driveWaypoint(flip, 7, blue))
                        .andThen(driveWaypoint(flip, 8, blue))
                        .andThen(driveWaypoint(flip, 9, blue))
                        .andThen(driveWaypoint(flip, 10, blue))
                )
                .andThen(driveWaypoint(flip, 11, blue)),
            shootFuel()
        ).withName(getCommandName());
    }
}
