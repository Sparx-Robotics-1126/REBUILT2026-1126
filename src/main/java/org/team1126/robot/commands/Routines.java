package org.team1126.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.team1126.lib.math.geometry.ExtPose;
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

    public Command prepareForShooting(){
        return parallel(
                shooter.getReadyCommand()
                ).withName("Routines.prepareForShooting()");
    }
    public Command shootFuel() {
        return parallel(
                shootingLights(),
                storage.feedShooter(shooter::shooterIsReady),
                shooter.shoot(shooter::feederIsReady)).withName("Routines.score()");
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

    public Command lightsSolidRed() {
        return lights.top.setSolidRed().withName("Routines.lightsSolidRed()");
        // return lights.top.setSolidRed();
    }

    public Command shootingLights() {
        return parallel(
            lights.sides.chase(Lights.Color.SHOOTING),
            lights.top.convergeToMiddle(Lights.Color.SHOOTING)
        ).withName("Routines.shootingLights()");
    }

    public Command selfDriveLights() {
        return parallel(
            lights.sides.fade(Lights.Color.BLUE, Lights.Color.RED),
            lights.top.knightRider(Lights.Color.BLUE, Lights.Color.RED)
        ).withName("Routines.selfDriveLights()");
    }

    private Pose2d waypoint(WaypointHeading heading, boolean left) {
        final boolean blue = Alliance.isBlue();
        Pose2d waypoint;
        if (WaypointHeading.NORTH == heading) {
            waypoint = blue ? Field.WAYPOINT_FAR.getBlue(left) : Field.WAYPOINT_FAR.getRed(left);
        } else {
            waypoint = blue ? Field.WAYPOINT_NEAR.getBlue(left) : Field.WAYPOINT_NEAR.getRed(left);
        }
        SmartDashboard.putString(
            "Waypoint",
            "X: " + waypoint.getX() + ", Y: " + waypoint.getY() + ", rot: " + waypoint.getRotation()
        );
        return waypoint;
    }

    private Command driveWaypoint(WaypointHeading heading, boolean left) {
        final Pose2d waypoint = waypoint(heading, left);
        return swerve.apfDrive(() -> waypoint, waypointDecel, waypointTol);
    }

    private ExtPose endpoint(WaypointHeading heading) {
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
        return new ExtPose(endpoint);
    }

    private Command driveEndpoint(WaypointHeading heading) {
        ExtPose endpoint = endpoint(heading);
        return swerve.apfDrive(endpoint, waypointDecel, waypointTol);
    }
    public Command aimAtHub(final DoubleSupplier maxDeceleration) {
        return parallel(selfDriveLights(),swerve.aimAtHub(maxDeceleration));
    }
    public Command trenchNorthWest() {
        return parallel(selfDriveLights(),
                driveWaypoint(WaypointHeading.NORTH, true).andThen(driveEndpoint(WaypointHeading.NORTH)));
    }

    public Command trenchNorthEast() {
        return parallel(selfDriveLights(),driveWaypoint(WaypointHeading.NORTH, false).andThen(driveEndpoint(WaypointHeading.NORTH)));
    }

    public Command trenchSouthWest() {
        return parallel(selfDriveLights(),driveWaypoint(WaypointHeading.SOUTH, true).andThen(driveEndpoint(WaypointHeading.SOUTH)));
    }

    public Command trenchSouthEast() {
        return parallel(selfDriveLights(),driveWaypoint(WaypointHeading.SOUTH, false).andThen(driveEndpoint(WaypointHeading.SOUTH)));
    }

    /**
     * Method that allows us to go where we would expect to find fuel in the neutral zone.
     */
    public Command refuelFromNeutral() {
        return parallel(selfDriveLights(),
                swerve.apfDrive(
            () -> {
                return new Pose2d(
                    Field.CENTER_X - Units.inchesToMeters(35.95),
                    Field.CENTER_Y - Units.inchesToMeters(35.95),
                    Rotation2d.fromDegrees(0)
                );
            },
            () -> 0.3
        ));
    }

    public Command refuelFromDepot() {
        return parallel(selfDriveLights(),
                    swerve.apfDrive(() -> new Pose2d(Field.DEPOT_X, Field.DEPOT_Y, Field.DEPOT_ROT), () -> 0.2));
    }
}
