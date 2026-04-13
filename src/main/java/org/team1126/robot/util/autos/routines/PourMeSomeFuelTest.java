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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class PourMeSomeFuelTest extends BaseAutosRoutine {
    
    public static final String COMMAND_NAME = "ShootOutpostFuelTest.action";
    public static final String DISPLAY_NAME = "Pour Me Some Fuel TEST";
    public static final String ABBREVIATION = "PMSFT";

    public static final ExtPose STARTING_POINT = new ExtPose(AutosStart.BUMP.getStartingPoint(true, false));
    public static final Waypoint OUTPOST = new Waypoint(.302, .75, Math.toRadians(180.0), getDefaultDecel());
    public static final Waypoint PIVOT = new Waypoint(3.00, .7, Math.toRadians(180.0), getDefaultDecel());
    public static final Waypoint bumpPivot = new Waypoint(5.712, 2.5, Math.toRadians(0.0),getDefaultDecel());
    public static final Waypoint centerPoint = new Waypoint(8, 2.5, Math.toRadians(0.0),getDefaultDecel());

    private static PourMeSomeFuelTest instance;

    public static void init(Robot robot) {
        instance = new PourMeSomeFuelTest(COMMAND_NAME, DISPLAY_NAME, ABBREVIATION, robot);
    }

    public static PourMeSomeFuelTest get() {
        if (instance == null) {
            return null;
        }
        return instance;
    }

    private PourMeSomeFuelTest(String commandName, String displayName, String abbreviatedName, Robot robot) {
        super(commandName, displayName, abbreviatedName, robot);
        waypoints = new Waypoint[0];
    }

    public Command action(Supplier<AutosStart> startAt, Supplier<AutosFlip> flip, BooleanSupplier blue) {
        return sequence(
            atStartingPoint(() -> STARTING_POINT.get(blue.getAsBoolean(), flip.get().shouldFlip())),
            driveWaypoint(flip,PIVOT, blue),
            driveWaypoint(flip, OUTPOST, blue)
                .alongWith(robot.intake.extendIntake().withTimeout(2.0)))
                .andThen(Commands.waitSeconds(4),
            driveArchAndShootFuel().withDeadline(Commands.waitSeconds(10.0)),
            driveWaypoint(flip, bumpPivot, blue),
            driveWaypoint(flip, centerPoint, blue)
        ).withName(getCommandName());
    }
}
