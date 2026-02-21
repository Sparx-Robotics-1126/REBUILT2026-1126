package org.team1126.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.tunable.Tunables.TunableBoolean;
import org.team1126.lib.tunable.Tunables.TunableDouble;
import org.team1126.lib.util.Alliance;
import org.team1126.robot.Robot;
import org.team1126.robot.subsystems.Lights;
import org.team1126.robot.subsystems.Shooter;
import org.team1126.robot.subsystems.Storage;
import org.team1126.robot.subsystems.Swerve;
import org.team1126.robot.util.Field;
import org.team1126.robot.util.WaypointNavigator.WaypointHeading;

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
        return parallel(storage.feedShooter(), shooter.feedShooter()).withName("Routines.score()");
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

    private Pose2d waypoint(WaypointHeading heading, boolean left) {
        final boolean blue = Alliance.isBlue();
        Pose2d waypoint;
        if (WaypointHeading.NORTH == heading) {
            waypoint = blue ? Field.WAYPOINT_NEAR.getBlue(left) : Field.WAYPOINT_NEAR.getRed(left);
        } else {
            waypoint = blue ? Field.WAYPOINT_FAR.getBlue(left) : Field.WAYPOINT_FAR.getRed(left);
        }
        SmartDashboard.putString(
            "Waypoint",
            "X: " + waypoint.getX() + ", Y: " + waypoint.getY() + ", rot: " + waypoint.getRotation()
        );
        return waypoint;
    }

    private Command driveWaypoint(WaypointHeading heading, boolean left) {
        final Pose2d waypoint = waypoint(heading, left);
        return swerve.apfDrive(() -> waypoint, waypointDecel::get);
    }

    private WaypointHeading findHeading() {
        WaypointHeading heading = WaypointHeading.NORTH;
        final boolean blue = Alliance.isBlue();
        double currentX = swerve.getPose().getX();
        SmartDashboard.putString("Heading", "NORTH");
        if (
            (blue && currentX > Field.BARRIER_FAR_RIGHT_CORNER.getBlue().getX())
            || (!blue && currentX < Field.BARRIER_FAR_RIGHT_CORNER.getRed().getX())
        ) {
            heading = WaypointHeading.SOUTH;
            SmartDashboard.putString("Heading", "SOUTH");
        }

        return heading;
    }

    private Pose2d endpoint(WaypointHeading heading) {
        final boolean blue = Alliance.isBlue();
        Pose2d endpoint;
        if (WaypointHeading.NORTH == heading) {
            endpoint = blue ? Field.WAYPOINT_GOAL_FAR.getBlue() : Field.WAYPOINT_GOAL_FAR.getRed();
        } else {
            endpoint = blue ? Field.WAYPOINT_GOAL_NEAR.getBlue() : Field.WAYPOINT_GOAL_NEAR.getRed();
        }
        SmartDashboard.putBoolean("Blue", blue);
        SmartDashboard.putString(
            "Endpoint",
            "X: " + endpoint.getX() + ", Y: " + endpoint.getY() + ", rot: " + endpoint.getRotation()
        );
        return endpoint;
    }

    private Command driveEndpoint(WaypointHeading heading) {
        Pose2d endpoint = endpoint(heading);
        return swerve.apfDrive(() -> endpoint, waypointDecel::get);
    }

    public Command trench(BooleanSupplier left) {
        final WaypointHeading heading = findHeading();
        return sequence(
            driveWaypoint(heading, left.getAsBoolean()).onlyWhile(
                () ->
                    swerve
                        .getPose()
                        .getTranslation()
                        .getDistance(waypoint(heading, left.getAsBoolean()).getTranslation())
                    > 0.1
            ),
            // driveWaypoint(WaypointHeading.SOUTH, left.getAsBoolean()).onlyWhile(
            //     () ->
            //         swerve
            //             .getPose()
            //             .getTranslation()
            //             .getDistance(waypoint(WaypointHeading.SOUTH, left.getAsBoolean()).getTranslation())
            //         > 0.1
            // ),
            driveEndpoint(heading)
        );
    }

    /**
     * Method that allows us to go where we would expect to find fuel in the neutral zone.
     */
    public Command refuelFromNeutral() {
        return swerve.apfDrive(
            () -> {
                return new Pose2d(
                    Field.CENTER_X - Units.inchesToMeters(35.95),
                    Field.CENTER_Y - Units.inchesToMeters(35.95),
                    Rotation2d.fromDegrees(0)
                );
            },
            () -> 0.3
        );
    }

    public Command refuelFromDepot() {
        return swerve.apfDrive(() -> new Pose2d(Field.DEPOT_X, Field.DEPOT_Y, Field.DEPOT_ROT), () -> 0.2);
    }
}
