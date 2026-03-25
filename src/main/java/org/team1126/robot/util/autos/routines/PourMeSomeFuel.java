package org.team1126.robot.util.autos.routines;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.team1126.robot.Robot;
import org.team1126.robot.util.autos.AutosFlip;
import org.team1126.robot.util.autos.AutosStart;
import org.team1126.robot.util.autos.BaseAutosRoutine;
import org.team1126.robot.util.nav.Waypoint;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class PourMeSomeFuel extends BaseAutosRoutine {
    
    public static final String COMMAND_NAME = "ShootOutpostFuel.action";
    public static final String DISPLAY_NAME = "Pour Me Some Fuel";
    public static final String ABBREVIATION = "PMSA";

    private static PourMeSomeFuel instance;

    public static void init(Robot robot) {
        instance = new PourMeSomeFuel(COMMAND_NAME, DISPLAY_NAME, ABBREVIATION, robot);
    }

    public static PourMeSomeFuel get() {
        if (instance == null) {
            return null;
        }
        return instance;
    }

    private PourMeSomeFuel(String commandName, String displayName, String abbreviatedName, Robot robot) {
        super(commandName, displayName, abbreviatedName, robot);
        waypoints = Arrays.asList(
            new Waypoint(3.028, 2.425, Math.toRadians(40), getDefaultDecel()),
            new Waypoint(1.994, 1.651, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(1.215, 0.453, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(0.500, 0.453, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(1.215, 0.453, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(1.994, 1.651, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(3.028, 2.425, Math.toRadians(40), getDefaultDecel())
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
                .andThen(Commands.waitSeconds(2.0))
                .andThen(driveWaypoint(flip, 4, blue))
                .andThen(driveWaypoint(flip, 5, blue))
                .andThen(driveWaypoint(flip, 6, blue)),
            driveArchAndShootFuel()
        ).withName(getCommandName());
    }
}
