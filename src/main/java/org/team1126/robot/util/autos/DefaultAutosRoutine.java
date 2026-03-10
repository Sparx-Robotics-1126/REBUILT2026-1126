package org.team1126.robot.util.autos;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.tunable.Tunables.TunableBoolean;
import org.team1126.lib.tunable.Tunables.TunableDouble;
import org.team1126.robot.Robot;
import org.team1126.robot.commands.Routines;
import org.team1126.robot.subsystems.Swerve;
import org.team1126.robot.util.nav.NorthDefaultNavigator;

/**
 * Default implementation of the AutosRoutine interface.
 */
public abstract class DefaultAutosRoutine extends NorthDefaultNavigator implements AutosRoutine {

    protected static final TunableTable tunables = Tunables.getNested("Robot Autos Defaults");
    protected static final TunableDouble flywheelWarmupTimer = tunables.value("Flywheel Warmup Timer", 0.1);
    protected static final TunableDouble secondsPerBall = tunables.value("Seconds per Ball (shooting)", 0.75);
    protected static final TunableDouble simulationDecel = tunables.value("Simulation Default Deceleration", 20.0);
    protected static final TunableDouble defaultDecel = tunables.value("Robot Autos Default Deceleration", 1.3);
    protected static final TunableDouble intakeFactor = tunables.value("Intake factor on set deceleration", 0.75);
    protected static final TunableBoolean limitedField = tunables.value(
        "Running full field (false at home, true at competitions)",
        false
    );

    /**
     * Sets the decel based on whether we are in a simulator instance or on the real robot.
     */
    public static double getDefaultDecel() {
        return RobotBase.isSimulation() ? simulationDecel.get() : defaultDecel.get();
    }

    /** This is final since we do not want this to be changes after instantiation */
    protected final String commandName;

    /** This is final since we do not want this to be changes after instantiation */
    protected final String displayName;

    protected final Routines routines = Routines.getInstance();

    // TODO: design this better when there is more time.
    protected final Robot robot;

    protected DefaultAutosRoutine(String commandName, String displayName, Swerve swerve, Robot robot) {
        this.commandName = commandName;
        this.displayName = displayName;
        this.swerve = swerve;
        this.robot = robot;
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

    public String getDisplayName(AutosStart startAt, AutosFlip flip) {
        return startAt + displayName + flip.display();
    }

    /**
     * Default implementaiton to prevent errors since the action method will be
     * mutually exclusive if there is a starting point.
     *
     * @param flip ignored
     * @returns the none command.
     */
    public Command action(AutosFlip flip) {
        return Commands.none();
    }

    /**
     * Default implementaiton to prevent errors since the action method will be
     * mutually exclusive if there is a starting point.
     *
     * @param startAt ignored
     * @param flip ignored
     * @returns the none command.
     */
    public Command action(AutosStart startAt, AutosFlip flip) {
        return Commands.none();
    }
}
