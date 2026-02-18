package org.team1126.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static edu.wpi.first.wpilibj2.command.Commands.deadline;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.tunable.Tunables.TunableBoolean;
import org.team1126.lib.tunable.Tunables.TunableDouble;
import org.team1126.robot.Robot;
import org.team1126.robot.subsystems.Lights;
import org.team1126.robot.subsystems.Shooter;
import org.team1126.robot.subsystems.Storage;
import org.team1126.robot.subsystems.Swerve;
import org.team1126.robot.util.Field;
import org.team1126.robot.util.Field.WaypointHeading;

/**
 * The Routines class contains command compositions, such as sequences
 * or parallel command groups, that require multiple subsystems.
 */
@SuppressWarnings("unused")
public final class Routines {

    private static final TunableTable tunables = Tunables.getNested("routines");

    private static final TunableDouble waypointDecel = tunables.value("waypointDecel", 12.0);
    private static final TunableDouble waypointTol = tunables.value("waypointTol", 0.25);

    private static final TunableBoolean autoDrive = tunables.value("autoDrive", true);

    private final Robot robot;

    private final Lights lights;
    private final Swerve swerve;
    private final Shooter shooter;
    private final Storage storage;

    //    private final ReefSelection selection;

    public Routines(Robot robot) {
        this.robot = robot;
        lights = robot.lights;
        swerve = robot.swerve;
        shooter = robot.shooter;
        storage = robot.storage;
        //        selection = robot.selection;
    }

    public Command shootFuel() {
        return sequence(
            //this readies shooter
            deadline(shooter.readyShooter(), waitUntil(shooter::isReady)),
            //this shoots fuel
            parallel(storage.feedShooter(), shooter.feedShooter())
        ).withName("Routines.score()");
    }

    public Command releaseAll() {
        return parallel(storage.spill()).withName("Spill Fuel");
    }

    /**
     * Displays the pre-match animation.
     * @param defaultAutoSelected If the default auto is selected.
     */
    public Command lightsPreMatch(BooleanSupplier defaultAutoSelected) {
        return lights
            .preMatch(swerve::getPose, swerve::seesAprilTag, defaultAutoSelected::getAsBoolean)
            .withName("Routines.lightsPreMatch()");
    }

    private Command waypoint(WaypointHeading heading, boolean left) {
        switch (heading) {
            case NORTH:
                return swerve.apfDrive(() -> Field.WAYPOINT_FAR.get(left), waypointDecel::get, waypointTol::get);
            case SOUTH:
                return swerve.apfDrive(() -> Field.WAYPOINT_NEAR.get(left), waypointDecel::get, waypointTol::get);
            default:
                return null;
        }
    }

    private Command endpoint(WaypointHeading heading) {
        switch (heading) {
            case NORTH:
                return swerve.apfDrive(() -> Field.WAYPOINT_GOAL_FAR.get(), waypointDecel::get, waypointTol::get);
            case SOUTH:
                return swerve.apfDrive(() -> Field.WAYPOINT_GOAL_FAR.get(), waypointDecel::get, waypointTol::get);
            default:
                return null;
        }
    }

    public Command trench(BooleanSupplier left) {
        // compound command that will
        // 1. determine where the robot is on the field
        // 2. determine the intermediate waypoint based on where we are and which trench we want to go through
        // 3. go to the waypoint
        // 4. go to the goal point
        return sequence(
            waypoint(WaypointHeading.NORTH, left.getAsBoolean()),
            endpoint(WaypointHeading.NORTH),
            swerve.stop(true)
        );
    }

    /**
     * Method that allows us to go where we would expect to find fuel in the neutral zone.
     */
    public Command refuelFromNeutral() {
        return swerve.apfDrive(
            () -> {
                return new Pose2d(
                    Field.X_CENTER - Units.inchesToMeters(35.95),
                    Field.Y_CENTER - Units.inchesToMeters(35.95),
                    Rotation2d.fromDegrees(0)
                );
            },
            () -> 0.3
        );
    }

    public Command refuelFromDepot() {
        return swerve.apfDrive(() -> new Pose2d(Field.DEPOT_X, Field.DEPOT_Y, Field.DEPOT_ROT), () -> 0.8);
    }
}
