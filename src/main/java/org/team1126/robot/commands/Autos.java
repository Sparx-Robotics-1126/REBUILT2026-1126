package org.team1126.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team1126.lib.math.geometry.ExtPose;
import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.tunable.Tunables.TunableDouble;
import org.team1126.robot.Robot;
import org.team1126.robot.subsystems.*;
import org.team1126.robot.util.Field;
import org.team1126.robot.util.autos.AutosFlip;
import org.team1126.robot.util.autos.routines.GrabAndShoot;
import org.team1126.robot.util.autos.routines.IntakeCenter;
import org.team1126.robot.util.autos.routines.SweepCenter;
import org.team1126.robot.util.nav.WaypointHeading;
import org.team1126.robot.util.nav.autos.ShootFirstAskQuestionsLaterAutosMap;

/**
 * The Autos class declares autonomous modes, and adds them
 * to the dashboard to be selected by the drive team.
 */
@SuppressWarnings("unused")
public final class Autos {

    private static final TunableTable tunables = Tunables.getNested("autos");
    private static final TunableDouble deceleration = tunables.value("deceleration", 6.0);
    private static final TunableDouble tolerance = tunables.value("tolerance", 0.05);

    private final Robot robot;

    private final Lights lights;
    private final Swerve swerve;
    private final Shooter shooter;
    private final Storage storage;
    private final Intake intake;

    private final Routines routines;

    SendableChooser<Command> chooser = new SendableChooser<>();

    public Autos(Robot robot) {
        this.robot = robot;

        lights = robot.lights;
        swerve = robot.swerve;
        shooter = robot.shooter;
        storage = robot.storage;
        intake = robot.intake;

        routines = robot.routines;

        SweepCenter.init(swerve, robot);
        IntakeCenter.init(swerve, robot);
        GrabAndShoot.init(swerve, robot);

        AutosFlip right = AutosFlip.RIGHT;
        AutosFlip left = AutosFlip.LEFT;
        ShootFirstAskQuestionsLaterAutosMap.init(swerve);

        // Create the auto chooser
        // chooser = new AutoChooser();
        chooser.setDefaultOption("Do nothing", Commands.none());
        chooser.addOption("Just Shoot", justShoot());
        // chooser.addOption("Drive", driveSampleLocations());
        chooser.addOption("Outpost", outpost());
        chooser.addOption("Trench", driveToFuel());
        chooser.addOption(SweepCenter.get().getDisplayName(right), SweepCenter.get().action(right));
        chooser.addOption(SweepCenter.get().getDisplayName(left), SweepCenter.get().action(left));
        chooser.addOption(IntakeCenter.get().getDisplayName(right), IntakeCenter.get().action(right));
        chooser.addOption(IntakeCenter.get().getDisplayName(left), IntakeCenter.get().action(left));
        chooser.addOption(GrabAndShoot.get().getDisplayName(right), GrabAndShoot.get().action(right));
        chooser.addOption(GrabAndShoot.get().getDisplayName(left), GrabAndShoot.get().action(left));
        // chooser.addOption("Depot", routines.dock());
        SmartDashboard.putData("autos", chooser);
    }

    public Command outpost() {
        var goal = Field.WAYPOINT_DEPOT.get();

        return parallel(
            routines.shootingLights(),
            sequence(
                swerve.resetPose(new ExtPose(2.287, 4.037, Rotation2d.kZero)),
                swerve.driveToShootingArc(() -> 0.8).withTimeout(1),
                routines.readyFeederShooter().withTimeout(.10),
                routines.shootFuelAuto().withTimeout(8.0),
                parallel(routines.fuelFromOutpost().withTimeout(5.0))
            )
        ).withName("Autos.outpost()");

        // deadline(routines.selfDriveLights(), shooter.readyShooter()),
        // // routines.shootFuel().withTimeout(3.0),
        // deadline(
        //     swerve.apfDrive(() -> new Pose2d(goal.getX(), goal.getY(), Rotation2d.fromDegrees(180)), () -> 0.3),
        //     intake.extendIntake()
        // )
    }

    public Command justShoot() {
        var goal = Field.WAYPOINT_DEPOT.get();

        return parallel(
            routines.shootingLights(),
            sequence(
                swerve.resetPose(new ExtPose(2.287, 4.037, Rotation2d.kZero)),
                swerve.driveToShootingArc(() -> 0.8).withTimeout(1),
                routines.readyFeederShooter().withTimeout(.10),
                routines.shootFuelAuto().withTimeout(20.0)
            )
        ).withName("Autos.justShoot()");

        // deadline(routines.selfDriveLights(), shooter.readyShooter()),
        // // routines.shootFuel().withTimeout(3.0),
        // deadline(
        //     swerve.apfDrive(() -> new Pose2d(goal.getX(), goal.getY(), Rotation2d.fromDegrees(180)), () -> 0.3),
        //     intake.extendIntake()
        // )
    }

    public Command driveToFuel() {
        var goal = Field.WAYPOINT_DEPOT.get();

        return parallel(
            routines.shootingLights(),
            sequence(
                swerve.resetPose(new ExtPose(2.287, 4.037, Rotation2d.kZero)),
                swerve.driveToShootingArc(() -> 0.8).withTimeout(1),
                routines.readyFeederShooter().withTimeout(.10),
                routines.shootFuelAuto().withTimeout(8.0),
                routines.driveTrench(() -> WaypointHeading.NORTH, () -> true)
            )
        ).withName("Autos.driveToFuel()");
    }

    /**
     * Starting point: Just south of blue line in line with trench
     * Step 1: Parallel:
     *            * Activate intake
     *            * Drive to turning point
     * Step 2: Pick up balls to center line
     * Step 3: Turn toward hub, come back through same side
     * Step 4: Shoot
     *
     * @param left
     * @return
     */
    public Command shootFirstAskQuestionsLater(boolean left) {
        // 3.539, 0.625, 0.00
        WaypointHeading heading = WaypointHeading.NORTH;
        return sequence(
            swerve.resetPose(new ExtPose(3.539, 0.625, Rotation2d.kZero)),
            ShootFirstAskQuestionsLaterAutosMap.get()
                .heading(heading)
                .andThen(
                    sequence(
                        parallel(
                            intake.extendIntake(false).withTimeout(1.5).andThen(intake.moveIntakeMotorCommand(false)),
                            Commands.waitSeconds(2.0)
                                .andThen(
                                    ShootFirstAskQuestionsLaterAutosMap.get().driveWaypoint(heading, () -> left, 0)
                                )
                                .andThen(
                                    ShootFirstAskQuestionsLaterAutosMap.get().driveWaypoint(heading, () -> left, 1)
                                )
                                .andThen(
                                    ShootFirstAskQuestionsLaterAutosMap.get().driveWaypoint(heading, () -> left, 2)
                                )
                                .andThen(
                                    ShootFirstAskQuestionsLaterAutosMap.get().driveWaypoint(heading, () -> left, 3)
                                )
                        ),
                        ShootFirstAskQuestionsLaterAutosMap.get()
                            .driveWaypoint(heading, () -> left, 4)
                            .andThen(ShootFirstAskQuestionsLaterAutosMap.get().driveWaypoint(heading, () -> left, 5))
                            .andThen(
                                parallel(
                                    ShootFirstAskQuestionsLaterAutosMap.get()
                                        .driveWaypoint(heading, () -> left, 6)
                                        .andThen(
                                            ShootFirstAskQuestionsLaterAutosMap.get().driveWaypoint(
                                                heading,
                                                () -> left,
                                                7
                                            )
                                        )
                                        .andThen(
                                            ShootFirstAskQuestionsLaterAutosMap.get().driveWaypoint(
                                                heading,
                                                () -> left,
                                                8
                                            )
                                        ),
                                    routines.readyFeederShooter().withTimeout(1.00).andThen(routines.shootFuelAuto())
                                )
                            )
                    )
                )
        ).withName("Autos.intakeCenter()");
    }

    /**
     * Returns {@code true} when the default auto is selected.
     */
    //    public boolean defaultSelected() {
    //        return chooser.getSelected().getAsBoolean();
    //    }
    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }

    private Command driveSampleLocations() {
        var start = new ExtPose(2.0, 2.0, Rotation2d.kZero);
        var middle = new ExtPose(3.0, 5.0, Rotation2d.k180deg);
        var end = new ExtPose(6.0, 2.5, Rotation2d.kCW_90deg);

        return sequence(
            swerve.resetPose(start),
            swerve.apfDrive(middle, deceleration, tolerance),
            swerve.apfDrive(end, deceleration, tolerance),
            swerve.stop(false)
        );
    }

    public Command runSelectedAuto() {
        System.out.println("Running auto: " + chooser.getSelected().getName());
        return chooser.getSelected();
    }

    // ********** Sim / Testing **********
}
