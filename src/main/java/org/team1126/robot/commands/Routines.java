package org.team1126.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.race;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.tunable.Tunables.TunableBoolean;
import org.team1126.lib.tunable.Tunables.TunableDouble;
import org.team1126.lib.tunable.Tunables.TunableInteger;
import org.team1126.lib.util.Alliance;
import org.team1126.robot.Robot;
import org.team1126.robot.subsystems.Feeder;
import org.team1126.robot.subsystems.Hood;
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

    public static void init(Robot robot) {
        if (instance == null) {
            instance = new Routines(robot);
        }
    }

    public static Routines getInstance() {
        return instance;
    }

    private static Routines instance;

    private static final TunableTable tunables = Tunables.getNested("routines");

    private static final TunableDouble waypointDecel = tunables.value("waypointDecel", 0.4);
    private static final TunableDouble waypointTol = tunables.value("waypointTol", 0.25);

    private static final TunableBoolean autoDrive = tunables.value("autoDrive", true);
    private static final TunableDouble staticShootDistance = tunables.value("staticShootDistance", 2.49);
    private static final TunableDouble staticShootHoodPosition = tunables.value("staticShootHoodPosition", 6.0);

    private static final TunableInteger shootingMinRqTagsSeen = tunables.value("shootingMinRqTagsSeen", 25);
    private final Robot robot;

    private final Lights lights;
    private final Swerve swerve;
    private final Shooter shooter;
    private final Hood hood;
    private final Storage storage;
    private final Intake intake;
    private final Feeder feeder;

    private Routines(Robot robot) {
        this.robot = robot;
        lights = robot.lights;
        swerve = robot.swerve;
        shooter = robot.shooter;
        hood = robot.hood;
        storage = robot.storage;
        intake = robot.intake;
        feeder = robot.feeder;
        //        selection = robot.selection;
    }

    public Command fuelFromOutpost() {
        var goal = Field.WAYPOINT_DEPOT.get();
        return parallel(
            swerve.apfDrive(() -> new Pose2d(goal.getX(), goal.getY(), Rotation2d.fromDegrees(180)), () -> 0.3),
            intake.extendIntake().withTimeout(3.0)
        ).withName("Routines.fuelFromOutpost()");
    }

    public Command lightsDisabledMode() {
        if (Alliance.isBlue()) {
            return parallel(lights.topLeftTop.setSolidBlue(), lights.topLeftBottom.setSolidBlue(), lights.sides.setSolidBlue(), lights.topRightBottom.setSolidBlue(), lights.topRightTop.setSolidBlue()).withName(
                "Routines.lightsDisabledMode()"
            );
        } else {
            return parallel(lights.topLeftTop.setSolidRed(), lights.topLeftBottom.setSolidRed(), lights.sides.setSolidRed(), lights.topRightBottom.setSolidRed(), lights.topRightTop.setSolidRed()).withName(
                "Routines.lightsDisabledMode()"
            );
        }
    }

    public Command lightsTeleopMode() {
        if (Alliance.isBlue()) {
            return parallel(lights.topLeftTop.setSolidBlue(), lights.topLeftBottom.setSolidBlue(), lights.sides.setSolidBlue(), lights.topRightBottom.setSolidBlue(), lights.topRightTop.setSolidBlue()).withName(
                "Routines.lightsDisabledMode()"
            );
        } else {
            return parallel(lights.topLeftTop.setSolidRed(), lights.topLeftBottom.setSolidRed(), lights.sides.setSolidRed(), lights.topRightBottom.setSolidRed(), lights.topRightTop.setSolidRed()).withName(
                "Routines.lightsDisabledMode()"
            );
        }
    }

    public Command driveDepot() {
        // var debot = Field.WAYPOINT_DEPOT.get();
        var goal = new Pose2d(
            Field.WAYPOINT_DEPOT.get().getX(),
            Field.WAYPOINT_DEPOT.get().getY(),
            Rotation2d.fromDegrees(-177)
        );
        return sequence(swerve.apfDrive(() -> goal, () -> 0.3)).withName("Routines.driveOutpost()");
    }

    // public Command outpost() {
    //     return sequence(
    //         selfDriveLights(),
    //         // readyFeederShooter(),
    //         shootFuel(),
    //         waitSeconds(3),
    //         swerve.apfDrive(() -> new Pose2d(1.388, 5.717, Rotation2d.fromDegrees(0)), () -> 0.3)
    //     ).withName("Routines.outpost()");
    // }

    // public Command prepareForShooting() {
    //     return parallel(shooter.getReadyCommand()).withName("Routines.prepareForShooting()");
    // }

    // public Command feedShooter() {
    //     return parallel(shootingLights(), storage.feedShooter(shooter::shooterIsReady), feeder.feedShooter()).withName(
    //         "Routines.score()"
    //     );
    // }

    // TODO: Implement this with setpoints and all that
    // public Command newShootFuel() {
    //     return parallel(
    //         hood.targetDistance(swerve::targetDistance),
    //         shooter.targetDistance(swerve::targetDistance),)
    // }

 /**
     * Shoots at the hub, without commanding the drivetrain.
     * @param runIntake Whether the intake should also be intaking.
     * @param force A supplier that if {@code true} will force the indexer to feed the shooters.
     */
    public Command shoot(BooleanSupplier runIntake, BooleanSupplier force) {
        return parallel(
            hood.targetDistance(swerve::distanceToTarget),
            shooter.targetDistance(swerve::distanceToTarget),
            // feeder.readyFeeder(),
            storage.feedShooter(()-> true),
            sequence(
                sequence(
                    waitSeconds(0.05),
                    waitUntil(
                        () ->
                            (hood.atPosition()
                                && shooter.shooterIsReady()
                                && swerve.aimingAtTarget()
                                && swerve.tagsSeen() >= shootingMinRqTagsSeen.get())
                            || force.getAsBoolean()
                    )
                ).deadlineFor(storage.spill().withTimeout(0.25)),
                feeder.feedShooter(()-> true),
                storage.feedShooter(()-> true)
            ),
            sequence(
                race(waitUntil(runIntake), waitSeconds(0.75)),
                either(sequence(intake.extendIntake(), intake.moveIntake(true)).onlyWhile(runIntake), intake.agitate().until(runIntake), runIntake)
            ).repeatedly()
        ).withName("Routines.shoot()");
    }

    /**
     * Shoots at the hub from a fixed distance, as a backup.
     */
    public Command staticShoot() {
        return parallel(
            hood.targetDistance(staticShootHoodPosition),
            shooter.targetDistance(staticShootDistance),
            feeder.feedShooter(() -> true),
            storage.feedShooter(feeder::isReady)
        ).withName("Routines.shoot()");
    }


    public Command shootFuelAuto() {
        return parallel(
            //    lights.top.setSolidRed()
            shooter.targetDistance(swerve::distanceToTarget),
            storage.feedShooter(shooter::shooterIsReady)
        ).withName("Routines.scoreAuto()");
    }

public Command shootFuelTest(){
    return parallel(
        storage.feedShooter(() -> true),
        feeder.feedShooter(shooter::shooterIsReady  ),
         shooter.readyShooter()
        // shootingLights(),
        // lights.sides.chase(Lights.Color.SHOOTING),
        // lights.top.convergeToMiddle(Lights.Color.SHOOTING),
        // hood.targetDistance(swerve::distanceToTarget),
        // shooter.targetDistance(swerve::distanceToTarget),
        // sequence(waitUntil(() -> (shooter.shooterIsReady() && hood.atPosition() && swerve.aimingAtTarget())), parallel(feeder.feedShooter(), storage.feedShooter(() -> feeder.isReady())))
    ).withName("Routines.score()");
}

    // public Command shootFuel() {
    //     return parallel(
    //         // shootingLights(),
    //         lights.sides.chase(Lights.Color.SHOOTING),
    //         lights.top.convergeToMiddle(Lights.Color.SHOOTING),
    //         hood.targetDistance(swerve::distanceToTarget),
    //         shooter.targetDistance(swerve::distanceToTarget),
    //         sequence(waitUntil(() -> (shooter.shooterIsReady() && hood.atPosition() && swerve.aimingAtTarget())), parallel(feeder.feedShooter(), storage.feedShooter(() -> feeder.isReady())))
    //     ).withName("Routines.score()");
    // }


    public Command shootFuelRev() {
        return parallel(
            // shootingLights(),
            storage.feedShooter(shooter::shooterIsReady),
            shooter.targetDistance(swerve::distanceToTarget)
        ).withName("Routines.score()");
    }

    public Command shootFieldFuel() {
        return parallel(
            // shootingLights(),
            storage.feedShooter(shooter::shooterIsReady),
            shooter.targetDistance(swerve::distanceToTarget)
        ).withName("Routines.score()");
    }

    public Command shootFuelReverseStorage() {
        return parallel(
            shootingLights(),
            storage.feedShooterReverse(shooter::shooterIsReady),
             shooter.targetDistance(swerve::distanceToTarget)
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

    // public Command readyFeederShooter() {
    //     return parallel(
    //         // lights.aiming(),
             
    //         shooter.getReadyCommand()
    //     ).withName("Routines.readyFeederShooter()");
    // }

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
        return parallel(lights.topLeftTop.setSolidRed(), lights.topLeftBottom.setSolidRed(), lights.sides.setSolidRed(), lights.topRightBottom.setSolidRed(), lights.topRightTop.setSolidRed()).withName("Routines.lightsSolidRed()");
        // return lights.top.setSolidRed();
    }

    public Command shootingLights() {
        return parallel(
            // lights.sides.chase(Lights.Color.SHOOTING),
            lights.topLeftTop.convergeToMiddle(Lights.Color.SHOOTING),
            lights.topLeftBottom.convergeToMiddle(Lights.Color.SHOOTING),
            lights.topRightBottom.convergeToMiddle(Lights.Color.SHOOTING),
            lights.topRightTop.convergeToMiddle(Lights.Color.SHOOTING),
            lights.sides.chase(Lights.Color.SHOOTING)
        ).withName("Routines.shootingLights()");
    }

    public Command selfDriveLights() {
        return parallel(
            lights.sides.fade(Lights.Color.BLUE, Lights.Color.RED),
            lights.topLeftTop.knightRider(Lights.Color.BLUE, Lights.Color.RED),
            lights.topLeftBottom.knightRider(Lights.Color.BLUE, Lights.Color.RED),
            lights.topRightBottom.knightRider(Lights.Color.BLUE, Lights.Color.RED),
            lights.topRightTop.knightRider(Lights.Color.BLUE, Lights.Color.RED)
        ).withName("Routines.selfDriveLights()");
    }

    public Command aimAtHub(final DoubleSupplier maxDeceleration) {
        return parallel(selfDriveLights(), swerve.aimAtHub(maxDeceleration));
    }

    public Command driveTrenchWithLights(Supplier<WaypointHeading> headingSupplier, BooleanSupplier left) {
        return parallel(selfDriveLights(), driveTrench(headingSupplier, left));
    }

    public Command driveTrench(Supplier<WaypointHeading> headingSupplier, BooleanSupplier left) {
        return TrenchNavigator.get()
            .heading(headingSupplier)
            .andThen(TrenchNavigator.get().driveWaypoint(headingSupplier, left, 0))
            .andThen(TrenchNavigator.get().driveWaypoint(headingSupplier, left, 1));
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
