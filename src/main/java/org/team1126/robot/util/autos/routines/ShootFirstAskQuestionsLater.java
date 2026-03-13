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
public final class ShootFirstAskQuestionsLater extends BaseAutosRoutine {

    public static final String COMMAND_NAME = "ShootNowAskQuestionsLater.action()";
    public static final String DISPLAY_NAME = "Just Shoot";
    public static final String ABBREVIATION = "PEWPEW";

    private static ShootFirstAskQuestionsLater instance;

    public static void init(Robot robot) {
        instance = new ShootFirstAskQuestionsLater(COMMAND_NAME, DISPLAY_NAME, ABBREVIATION, robot);
    }

    public static ShootFirstAskQuestionsLater get() {
        if (instance == null) {
            return null;
        }

        return instance;
    }

    /**
     * Singleton constructor
     */
    private ShootFirstAskQuestionsLater(String commandName, String displayName, String abbreviatedName, Robot robot) {
        super(commandName, displayName, abbreviatedName, robot);
        waypoints = Arrays.asList(new Waypoint(3.172, 3.232, Math.toRadians(0.0), getDefaultDecel())).toArray(
            new Waypoint[0]
        );
    }

    public Command action(Supplier<AutosStart> startAt, Supplier<AutosFlip> flip, BooleanSupplier blue) {
        return sequence(
            atStartingPoint(() -> startAt.get().getStartingPoint(blue.getAsBoolean(), flip.get().shouldFlip())),
            driveWaypoint(flip, 0, blue),
            driveArchAndShootFuelStart()
        ).withName(getCommandName());
    }
}
