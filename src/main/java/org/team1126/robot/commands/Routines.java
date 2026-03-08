package org.team1126.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.tunable.Tunables.TunableBoolean;
import org.team1126.lib.tunable.Tunables.TunableDouble;
import org.team1126.lib.util.Alliance;
import org.team1126.robot.Robot;
import org.team1126.robot.subsystems.Intake;
import org.team1126.robot.subsystems.Lights;
import org.team1126.robot.subsystems.Shooter;
import org.team1126.robot.subsystems.Storage;
import org.team1126.robot.subsystems.Swerve;
import org.team1126.robot.util.Field;
import org.team1126.robot.util.nav.TrenchNavigator;
import org.team1126.robot.util.nav.WaypointHeading;

/**
 * The Routines class contains command compositions, such as sequences
 * or parallel command groups, that require multiple subsystems.
 */
@SuppressWarnings("unused")
public final class Routines {

    private static final TunableTable tunables = Tunables.getNested("routines");

    private static final TunableDouble waypointDecel = tunables.value("waypointDecel", 0.4);
    private static final TunableDouble waypointTol = tunables.value("waypointTol", 0.25);

    private static final TunableBoolean autoDrive = tunables.value("autoDrive", true);

    private final Robot robot;

    private final Lights lights;
    private final Swerve swerve;
    private final Shooter shooter;
    private final Storage storage;
    private final Intake intake;

    //    private final ReefSelection selection;

    public Routines(Robot robot) {
        this.robot = robot;
        lights = robot.lights;
        swerve = robot.swerve;
        shooter = robot.shooter;
        storage = robot.storage;
        intake = robot.intake;
        //        selection = robot.selection;
    }

    public Command fuelFromOutpost() {
        var goal = Field.WAYPOINT_DEPOT.get();
        return parallel(
            swerve.apfDrive(() -> new Pose2d(goal.getX(), goal.getY(), Rotation2d.fromDegrees(180)), () -> 0.3),
            intake.extendIntake(true).withTimeout(3.0)
        ).withName("Routines.fuelFromOutpost()");
    }

    public Command lightsDisabledMode() {
        if (Alliance.isBlue()) {
            return parallel(lights.top.setSolidBlue(), lights.sides.setSolidBlue()).withName(
                "Routines.lightsDisabledMode()"
            );
        } else {
            return parallel(lights.top.setSolidRed(), lights.sides.setSolidRed()).withName(
                "Routines.lightsDisabledMode()"
            );
        }
    }

    public Command lightsTeleopMode() {
        if (Alliance.isBlue()) {
            return parallel(lights.top.setSolidBlue(), lights.sides.setSolidBlue()).withName(
                "Routines.lightsDisabledMode()"
            );
        } else {
            return parallel(lights.top.setSolidRed(), lights.sides.setSolidRed()).withName(
                "Routines.lightsDisabledMode()"
            );
        }
    }

    public Command driveOutpost() {
        // var debot = Field.WAYPOINT_DEPOT.get();
        var goal = new Pose2d(
            Field.WAYPOINT_DEPOT.get().getX(),
            Field.WAYPOINT_DEPOT.get().getY(),
            Rotation2d.fromDegrees(-177)
        );
        return sequence(swerve.apfDrive(() -> goal, () -> 0.3)).withName("Routines.driveOutpost()");
    }

    public Command outpost() {
        return sequence(
            selfDriveLights(),
            readyFeederShooter(),
            shootFuel(),
            waitSeconds(3),
            swerve.apfDrive(() -> new Pose2d(1.388, 5.717, Rotation2d.fromDegrees(0)), () -> 0.3)
        ).withName("Routines.outpost()");
    }

    public Command prepareForShooting() {
        return parallel(shooter.getReadyCommand()).withName("Routines.prepareForShooting()");
    }

    public Command feedShooter() {
        return parallel(shootingLights(), storage.feedShooter(shooter::shooterIsReady), shooter.feedShooter()).withName(
            "Routines.score()"
        );
    }

    public Command shootFuelAuto() {
        return parallel(
            // shootingLights(),
            shooter.shoot(shooter::feederIsReady),
            storage.feedShooter(shooter::shooterIsReady)
        ).withName("Routines.score()");
    }

    public Command shootFuel() {
        return parallel(
            // shootingLights(),
            storage.feedShooter(shooter::shooterIsReady),
            shooter.shoot(shooter::feederIsReady)
        ).withName("Routines.score()");
    }

    public Command shootFuelRev() {
        return parallel(
            shootingLights(),
            storage.feedShooter(shooter::shooterIsReady),
            shooter.shoot(shooter::feederIsReady)
        ).withName("Routines.score()");
    }

    public Command shootFieldFuel() {
        return parallel(
            // shootingLights(),
            storage.feedShooter(shooter::shooterIsReady),
            shooter.shootField(shooter::feederIsReady)
        ).withName("Routines.score()");
    }

    public Command shootFuelReverseStorage() {
        return parallel(
            shootingLights(),
            storage.feedShooterReverse(shooter::shooterIsReady),
            shooter.shoot(shooter::feederIsReady)
        ).withName("Routines.score()");
    }

    // public Command shootFuelReverseStorage() {
    //     return parallel(
    //         shootingLights(),
    //         storage.feedShooter(shooter::shooterIsReady),
    //         shooter.shoot(shooter::feederIsReady),
    //         storage.revStorage()
    //     ).withName("Routines.score()");
    // }

    // public Command unJamFeederShooter() {
    //     return parallel(
    //         shooter.unJamFeeder()
    //         // shooter.unJamShooter()
    //     ).withName("Routines.unJamFeederShooter()");
    // }

    public Command releaseAll() {
        return parallel(storage.spill()).withName("Spill Fuel");
    }

    public Command readyFeederShooter() {
        return parallel(
            shooter.getReadyCommand()
            //  storage.feedShooter(shooter::shooterIsReady)
        ).withName("Routines.readyFeederShooter()");
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

    public Command aimAtHub(final DoubleSupplier maxDeceleration) {
        return parallel(selfDriveLights(), swerve.aimAtHub(maxDeceleration));
    }

    public Command driveTrenchWithLights(Supplier<WaypointHeading> headingSupplier, BooleanSupplier left) {
        return parallel(selfDriveLights(), driveTrench(headingSupplier, left));
    }

    public Command driveTrench(Supplier<WaypointHeading> headingSupplier, BooleanSupplier left) {
        return TrenchNavigator.get()
            .heading(headingSupplier.get())
            .andThen(TrenchNavigator.get().driveWaypoint(headingSupplier.get(), 0))
            .andThen(TrenchNavigator.get().driveWaypoint(headingSupplier.get(), 0))
            .andThen(TrenchNavigator.get().driveWaypoint(headingSupplier.get(), 1))
            .andThen(TrenchNavigator.get().driveWaypoint(headingSupplier.get(), 1))
            .andThen(TrenchNavigator.get().driveWaypoint(headingSupplier.get(), 2))
            .andThen(TrenchNavigator.get().driveWaypoint(headingSupplier.get(), 2))
            .andThen(TrenchNavigator.get().driveWaypoint(headingSupplier.get(), 3))
            .andThen(TrenchNavigator.get().driveWaypoint(headingSupplier.get(), 3));
    }

    /**
     * Method that allows us to go where we would expect to find fuel in the neutral zone.
     */
    public Command refuelFromNeutral() {
        return parallel(
            selfDriveLights(),
            swerve.apfDrive(
                () -> {
                    return new Pose2d(
                        Field.CENTER_X - Units.inchesToMeters(35.95),
                        Field.CENTER_Y - Units.inchesToMeters(35.95),
                        Rotation2d.fromDegrees(0)
                    );
                },
                () -> 0.3
            )
        );
    }

    public Command refuelFromDepot() {
        return parallel(
            selfDriveLights(),
            swerve.apfDrive(() -> new Pose2d(Field.DEPOT_X, Field.DEPOT_Y, Field.DEPOT_ROT), () -> 0.2)
        );
    }
}
