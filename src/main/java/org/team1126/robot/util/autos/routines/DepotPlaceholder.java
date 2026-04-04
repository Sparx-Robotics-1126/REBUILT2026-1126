package org.team1126.robot.util.autos.routines;

import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import org.team1126.robot.Robot;
import org.team1126.robot.util.autos.AutosFlip;
import org.team1126.robot.util.autos.AutosStart;
import org.team1126.robot.util.autos.BaseAutosRoutine;
import org.team1126.robot.util.nav.Waypoint;

import edu.wpi.first.wpilibj2.command.Command;

public class DepotPlaceholder extends BaseAutosRoutine {
    
    public static final String COMMAND_NAME = "ShootDepotFuel.action";
    public static final String DISPLAY_NAME = "To Be Determined";
    public static final String ABBREVIATION = "TDB";

    private static DepotPlaceholder instance;

    public static void init(Robot robot) {
        instance = new DepotPlaceholder(COMMAND_NAME, DISPLAY_NAME, ABBREVIATION, robot);
    }

    public static DepotPlaceholder get() {
        if (instance == null) {
            return null;
        }
        return instance;
    }

    private DepotPlaceholder(String commandName, String displayName, String abbreviatedName, Robot robot) {
        super(commandName, displayName, abbreviatedName, robot);
        waypoints = Arrays.asList(
            new Waypoint(3.028, 5.116, Math.toRadians(40.0), getDefaultDecel()),
            new Waypoint(2.747, 5.630, Math.toRadians(90.0), getDefaultDecel()),
            new Waypoint(1.662, 5.856, Math.toRadians(180.0), getDefaultDecel()),
            new Waypoint(1.161, 5.856, Math.toRadians(180.0), getDefaultDecel() * intakeFactor.get()),
            new Waypoint(0.798, 5.856, Math.toRadians(180.0), getDefaultDecel() * intakeFactor.get()),
            new Waypoint(0.600, 5.856, Math.toRadians(180.0), getDefaultDecel() * intakeFactor.get()),
            new Waypoint(0.798, 5.856, Math.toRadians(180.0), getDefaultDecel()),
            new Waypoint(1.161, 5.856, Math.toRadians(180.0), getDefaultDecel()),
            new Waypoint(1.662, 5.856, Math.toRadians(90.0), getDefaultDecel()),
            new Waypoint(2.747, 5.630, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(3.028, 5.116, Math.toRadians(40.0), getDefaultDecel())
        ).toArray(new Waypoint[0]);
    }

    public Command action(Supplier<AutosStart> startAt, Supplier<AutosFlip> flip, BooleanSupplier blue) {
        Supplier<AutosFlip> invertedFlip = () -> {
            AutosFlip f = flip.get();
            if (f == AutosFlip.LEFT) return AutosFlip.RIGHT;
            if (f == AutosFlip.RIGHT) return AutosFlip.LEFT;
            return AutosFlip.NONE;
        };

        return sequence(
            atStartingPoint(() -> {
                AutosStart s = startAt.get();
                boolean left = s == AutosStart.BUMP ? true : !flip.get().shouldFlip();
                return s.getStartingPoint(blue.getAsBoolean(), left);
            }),
            driveWaypoint(invertedFlip, 0, blue),
            driveArchAndShootFuelStart(),
            driveWaypoint(invertedFlip, 1, blue)
                .andThen(driveWaypoint(invertedFlip, 2, blue))
                .andThen(
                    robot.intake
                        .extendIntake(false)
                        .withTimeout(0.5)
                        .andThen(robot.intake.moveIntakeTest(false))
                        .withDeadline(driveWaypoint(invertedFlip, 3, blue)
                        .andThen(driveWaypoint(invertedFlip, 4, blue))
                        .andThen(driveWaypoint(invertedFlip, 5, blue)))
                )
                .andThen(driveWaypoint(invertedFlip, 6, blue))
                .andThen(driveWaypoint(invertedFlip, 7, blue))
                .andThen(driveWaypoint(invertedFlip, 8, blue))
                .andThen(driveWaypoint(invertedFlip, 9, blue))
                .andThen(driveWaypoint(invertedFlip, 10, blue)),
            driveArchAndShootFuel()
        ).withName(getCommandName());
    }
}
