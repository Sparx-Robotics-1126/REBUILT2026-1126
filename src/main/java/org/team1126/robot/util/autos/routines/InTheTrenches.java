package org.team1126.robot.util.autos.routines;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.team1126.robot.Robot;
import org.team1126.robot.util.autos.AutosFlip;
import org.team1126.robot.util.autos.AutosStart;
import org.team1126.robot.util.autos.BaseAutosRoutine;
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
public final class InTheTrenches extends BaseAutosRoutine {

    public static final String COMMAND_NAME = "InTheTrenches.action()";
    public static final String DISPLAY_NAME = "Drive Trenches";
    public static final String ABBREVIATION = "TRENCHING";

    private static InTheTrenches instance;

    public static void init(Robot robot) {
        instance = new InTheTrenches(COMMAND_NAME, DISPLAY_NAME, ABBREVIATION, robot);
    }

    public static InTheTrenches get() {
        if (instance == null) {
            return null;
        }

        return instance;
    }

    /**
     * Singleton constructor
     */
    private InTheTrenches(String commandName, String displayName, String abbreviatedName, Robot robot) {
        super(commandName, displayName, abbreviatedName, robot);
        waypoints = Arrays.asList(
            new Waypoint(3.172, 3.232, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(2.975, 0.603, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(5.877, 0.603, Math.toRadians(0.0), getDefaultDecel()),
            new Waypoint(6.885, 1.899, Math.toRadians(60.0), getDefaultDecel())
        ).toArray(new Waypoint[0]);
    }

    public Command action(Supplier<AutosStart> startAt, Supplier<AutosFlip> flip, BooleanSupplier blue) {
        return parallel(
            routines.shootingLights(),
            sequence(
                atStartingPoint(() -> startAt.get().getStartingPoint(blue.getAsBoolean(), flip.get().shouldFlip())),
                driveWaypoint(flip, 0, blue),
                driveArchAndShootFuelStart(),
                driveWaypoint(flip, 1, blue),
                driveWaypoint(flip, 2, blue),
                driveWaypoint(flip, 3, blue)
            )
        ).withName(getCommandName());
    }
}
