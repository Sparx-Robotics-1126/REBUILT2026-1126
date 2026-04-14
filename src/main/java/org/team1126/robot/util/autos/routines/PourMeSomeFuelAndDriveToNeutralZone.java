package org.team1126.robot.util.autos.routines;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

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

public class PourMeSomeFuelAndDriveToNeutralZone extends BaseAutosRoutine {
    
    public static final String COMMAND_NAME = "ShootOutpostFuelAndDriveToNeutralZone.action";
    public static final String DISPLAY_NAME = "Pour Me Some Fuel and Drive To Neutral Zone";
    public static final String ABBREVIATION = "P-MID";

    public static final ExtPose STARTING_POINT = new ExtPose(AutosStart.BUMP.getStartingPoint(true, false));
    public static final Waypoint OUTPOST = new Waypoint(.302, .75, Math.toRadians(180.0), getDefaultDecel());
    public static final Waypoint PIVOT = new Waypoint(3.00, .7, Math.toRadians(180.0), getDefaultDecel());
    public static final Waypoint SHOOT_FROM_HERE = new Waypoint(2.74, 2.2, Math.toRadians(45.0), getDefaultDecel() * intakeFactor.get());
    public static final Waypoint NEUTRAL_ZONE = new Waypoint(7.6, 2.6, Math.toRadians(45.0), getDefaultDecel() * intakeFactor.get());

    private static PourMeSomeFuelAndDriveToNeutralZone instance;

    public static void init(Robot robot) {
        instance = new PourMeSomeFuelAndDriveToNeutralZone(COMMAND_NAME, DISPLAY_NAME, ABBREVIATION, robot);
    }

    public static PourMeSomeFuelAndDriveToNeutralZone get() {
        if (instance == null) {
            return null;
        }
        return instance;
    }

    private PourMeSomeFuelAndDriveToNeutralZone(String commandName, String displayName, String abbreviatedName, Robot robot) {
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
            driveWaypoint(flip, SHOOT_FROM_HERE, blue),
            routines.shoot(() -> false, () -> true).withTimeout(5.0),
            driveWaypoint(flip, NEUTRAL_ZONE, blue)
        ).withName(getCommandName());
    }
}
