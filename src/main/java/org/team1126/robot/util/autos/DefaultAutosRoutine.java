package org.team1126.robot.util.autos;

import edu.wpi.first.wpilibj.RobotBase;
import org.team1126.robot.commands.Routines;
import org.team1126.robot.subsystems.Swerve;
import org.team1126.robot.util.nav.NorthDefaultNavigator;
import org.team1126.robot.util.nav.Waypoint;

/**
 * Default implementation of the AutosRoutine interface.
 */
public abstract class DefaultAutosRoutine extends NorthDefaultNavigator implements AutosRoutine {

    public static final Waypoint LEFT_WAYPOINT = new Waypoint(2.54, 5.23, Math.toRadians(23.85), getDefaultDecel());
    public static final Waypoint CENTER_WAYPOINT = new Waypoint(2.287, 4.038, Math.toRadians(0.0), getDefaultDecel());
    public static final Waypoint RIGHT_WAYPOINT = new Waypoint(3.131, 2.228, Math.toRadians(51.13), getDefaultDecel());

    public static double getDefaultDecel() {
        return RobotBase.isSimulation() ? SIMULATION_DECEL : DEFAULT_DECEL;
    }

    /** This is final since we do not want this to be changes after instantiation */
    protected final String commandName;

    /** This is final since we do not want this to be changes after instantiation */
    protected final String displayName;

    protected final Routines routines = Routines.getInstance();

    protected DefaultAutosRoutine(String commandName, String displayName, Swerve swerve) {
        this.commandName = commandName;
        this.displayName = displayName;
        this.swerve = swerve;
    }

    public String getCommandName() {
        return commandName;
    }

    public String getDisplayName() {
        return displayName;
    }

    public String getDisplayName(AutosFlip flip) {
        return displayName + flip.display();
    }
}
