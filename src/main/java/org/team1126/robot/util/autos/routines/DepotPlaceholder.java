package org.team1126.robot.util.autos.routines;

import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import org.team1126.lib.math.geometry.ExtPose;
import org.team1126.robot.Robot;
import org.team1126.robot.util.autos.AutosFlip;
import org.team1126.robot.util.autos.AutosStart;
import org.team1126.robot.util.autos.BaseAutosRoutine;
import org.team1126.robot.util.nav.Waypoint;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class DepotPlaceholder extends BaseAutosRoutine {
    
    public static final String COMMAND_NAME = "ShootDepotFuel.action";
    public static final String DISPLAY_NAME = "To Be Determined";
    public static final String ABBREVIATION = "TBD";

    public static final ExtPose STARTING_POINT = new ExtPose(AutosStart.BUMP.getStartingPoint(true, false));
    public static final Waypoint DEPOT = new Waypoint(0.600, 5.856, Math.toRadians(180.0), getDefaultDecel() * intakeFactor.get());
    public static final Waypoint PIVOT_1 = new Waypoint(1.662, 5.856, Math.toRadians(180.0), getDefaultDecel());
    public static final Waypoint PIVOT_2 = new Waypoint(3.028, 5.116, Math.toRadians(40.0), getDefaultDecel());


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
        waypoints = new Waypoint[0];
    }

    public Command action(Supplier<AutosStart> startAt, Supplier<AutosFlip> flip, BooleanSupplier blue) {
        return sequence(
            atStartingPoint(() -> STARTING_POINT.get(blue.getAsBoolean(), flip.get().shouldFlip())),
            driveWaypoint(flip,PIVOT_1, blue),
            driveWaypoint(flip, DEPOT, blue),
            driveWaypoint(flip, PIVOT_2, blue)
                .alongWith(robot.intake.extendIntake().withTimeout(2.0)))
                .andThen(Commands.waitSeconds(4),
            driveArchAndShootFuel()
        ).withName(getCommandName());
    }
}
