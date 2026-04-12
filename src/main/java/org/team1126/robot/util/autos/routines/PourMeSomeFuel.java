package org.team1126.robot.util.autos.routines;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.team1126.lib.math.geometry.ExtPose;
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
    public static final String ABBREVIATION = "PMSF";

    public static final ExtPose STARTING_POINT = new ExtPose(AutosStart.BUMP.getStartingPoint(true, false));
    public static final Waypoint OUTPOST = new Waypoint(0.500, 0.453, Math.toRadians(180.0), getDefaultDecel());

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
        waypoints = new Waypoint[0];
    }

    public Command action(Supplier<AutosStart> startAt, Supplier<AutosFlip> flip, BooleanSupplier blue) {
        return sequence(
            atStartingPoint(() -> STARTING_POINT.get(blue.getAsBoolean(), flip.get().shouldFlip())),
            driveArchAndShootFuelStart(),
            driveWaypoint(flip, OUTPOST, blue)
                .andThen(robot.intake.extendIntake())
                .andThen(Commands.waitSeconds(2.0)),
            driveArchAndShootFuel()
        ).withName(getCommandName());
    }
}
