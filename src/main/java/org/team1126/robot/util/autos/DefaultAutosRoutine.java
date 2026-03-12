package org.team1126.robot.util.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.tunable.Tunables.TunableBoolean;
import org.team1126.lib.tunable.Tunables.TunableDouble;
import org.team1126.robot.Robot;
import org.team1126.robot.commands.Routines;
import org.team1126.robot.util.nav.Navigator;
import org.team1126.robot.util.nav.Waypoint;
import org.team1126.robot.util.nav.WaypointHeading;

/**
 * Default implementation of the AutosRoutine interface.
 */
public abstract class DefaultAutosRoutine implements AutosRoutine, Navigator {

    protected static final TunableTable tunables = Tunables.getNested("Robot Autos Defaults");
    protected static final TunableDouble flywheelWarmupTimer = tunables.value("Flywheel Warmup Timer", 0.1);
    protected static final TunableDouble secondsPerBall = tunables.value("Seconds per Ball (shooting)", 0.55);
    protected static final TunableDouble simulationDecel = tunables.value("Simulation Default Deceleration", 20.0);
    protected static final TunableDouble defaultDecel = tunables.value("Robot Autos Default Deceleration", 1.3);
    protected static final TunableDouble intakeTimer = tunables.value("Intake extension timer", 1.5);
    protected static final TunableDouble intakeFactor = tunables.value("Intake factor on set deceleration", 0.75);
    protected static final TunableBoolean limitedField = tunables.value(
        "Running full field (false at home, true at competitions)",
        true
    );
    // defaulting to 25 cm
    protected static final TunableDouble defaultTolerance = tunables.value(
        "Default swerve end tolerance (in meters)",
        0.25
    );
    protected static final TunableDouble defaultFuelLoad = tunables.value("Default fuel amount at start of autos", 8.0);
    protected static final TunableDouble maxShootingArcTravelTime = tunables.value(
        "Max amount of time to drive to shooting arc",
        1.0
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

    /** Abbreviated name for short displays. */
    protected final String abbreviatedName;

    protected final Routines routines = Routines.getInstance();

    // TODO: design this better when there is more time.
    protected final Robot robot;

    protected Waypoint[] waypoints;

    protected DefaultAutosRoutine(String commandName, String displayName, String abbreviatedName, Robot robot) {
        this.commandName = commandName;
        this.displayName = displayName;
        this.abbreviatedName = abbreviatedName;
        this.robot = robot;
    }

    public String getCommandName() {
        return commandName;
    }

    public String getDisplayName() {
        return displayName;
    }

    public String getDisplayName(boolean succinct) {
        return succinct ? abbreviatedName : displayName;
    }

    public String getDisplayName(AutosStart startAt) {
        return getDisplayName(startAt, false);
    }

    public String getDisplayName(AutosStart startAt, boolean succinct) {
        return getDisplayName(startAt, AutosFlip.NONE, succinct);
    }

    public String getDisplayName(AutosFlip flip) {
        return getDisplayName(flip, false);
    }

    public String getDisplayName(AutosFlip flip, boolean succinct) {
        return getDisplayName(AutosStart.OTHER, flip, succinct);
    }

    public String getDisplayName(AutosStart startAt, AutosFlip flip) {
        return getDisplayName(startAt, flip, false);
    }

    public String getDisplayName(AutosStart startAt, AutosFlip flip, boolean succinct) {
        String separator = succinct ? ":" : " - ";
        return (
            (succinct ? abbreviatedName : displayName)
            + (startAt != AutosStart.OTHER ? separator : "")
            + startAt.display(succinct)
            + (flip != AutosFlip.NONE ? separator : "")
            + flip.display(succinct)
        );
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

    public Command driveToShootingArch() {
        return robot.swerve.driveToShootingArc(defaultDecel);
    }

    protected Command shootFuel() {
        return sequence(
            routines.readyFeederShooter().withTimeout(flywheelWarmupTimer.getAsDouble()),
            routines.shootFuelAuto().withTimeout(secondsPerBall.getAsDouble() * defaultFuelLoad.getAsDouble())
        );
    }

    /**
     * This is meant to shoot fuel until autos are over, therefore no timeout. Hopefully we will have a good
     * amount of fuel in the hopper.
     *
     * @return the command to drive and shoot fuel.
     */
    protected Command driveArchAndShootFuel() {
        return sequence(
            parallel(
                routines.readyFeederShooter().withTimeout(flywheelWarmupTimer.getAsDouble()),
                driveToShootingArch().withTimeout(maxShootingArcTravelTime.getAsDouble())
            ),
            routines.shootFuelAuto()
        );
    }

    /**
     * This is meant to drive to the shooting arch and shoot the 8 balls that are known to be
     * in the hopper, it should timeout on that amount of fuel.
     *
     * @return the command to do this.
     */
    protected Command driveArchAndShootFuelStart() {
        return sequence(
            parallel(
                routines.readyFeederShooter().withTimeout(flywheelWarmupTimer.getAsDouble()),
                driveToShootingArch().withTimeout(maxShootingArcTravelTime.getAsDouble())
            ),
            routines.shootFuelAuto().withTimeout(secondsPerBall.getAsDouble() * defaultFuelLoad.getAsDouble())
        );
    }

    protected Command atStartingPoint(Pose2d startAt) {
        return atPoint(startAt);
    }

    protected Command atPoint(Pose2d point) {
        return robot.swerve.resetPose(() -> point);
    }

    /**
     * This is not specific for waypoint direction.
     *
     * @param flip
     * @param index
     * @return
     */
    public Command driveWaypoint(AutosFlip flip, int index) {
        if (index < waypoints.length) {
            Waypoint waypoint = waypoints[index];
            if (waypoint.limitField) {
                return Commands.none();
            }

            return robot.swerve.apfDrive(
                () -> waypoints[index].asPose(flip.shouldFlip()),
                () -> waypoints[index].decel,
                () -> defaultTolerance.getAsDouble()
            );
        } else {
            return Commands.none();
        }
    }

    /**
     * Not implemented.
     *
     * @return "none" command.
     */
    public Command driveWaypoint(WaypointHeading heading, BooleanSupplier left, int index) {
        return Commands.none();
    }

    /**
     * Not implemented.
     *
     * @return "none" command.
     */
    public Command heading(WaypointHeading heading) {
        return Commands.none();
    }
}
