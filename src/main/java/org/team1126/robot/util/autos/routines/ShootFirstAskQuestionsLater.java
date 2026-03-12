package org.team1126.robot.util.autos.routines;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1126.robot.Robot;
import org.team1126.robot.util.autos.AutosFlip;
import org.team1126.robot.util.autos.AutosStart;
import org.team1126.robot.util.autos.DefaultAutosRoutine;
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
public final class ShootFirstAskQuestionsLater extends DefaultAutosRoutine {

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
        waypoints = new Waypoint[0];
    }

    public Command action(AutosStart startAt, AutosFlip flip, boolean blue) {
        return sequence(atStartingPoint(startAt.getStartingPoint(blue)), driveArchAndShootFuelStart()).withName(
            getCommandName()
        );
    }
}
