package org.team1126.robot;

import com.ctre.phoenix6.Orchestra;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.team1126.lib.logging.LoggedRobot;
import org.team1126.lib.logging.Profiler;
import org.team1126.lib.util.DisableWatchdog;
import org.team1126.lib.util.vendors.PhoenixUtil;
import org.team1126.robot.commands.Autos;
import org.team1126.robot.commands.Routines;
import org.team1126.robot.subsystems.Intake;
import org.team1126.robot.subsystems.Lights;
import org.team1126.robot.subsystems.Shooter;
import org.team1126.robot.subsystems.Storage;
import org.team1126.robot.subsystems.Swerve;
import org.team1126.robot.util.MatchData;
import org.team1126.robot.util.WaypointNavigator.WaypointHeading;

@Logged
public final class Robot extends LoggedRobot {

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    public final Lights lights;
    public final Swerve swerve;
    public final Storage storage;
    public final Shooter shooter;
    public final Intake intake;

    public final MatchData matchData;

    public final Routines routines;
    public final Autos autos;

    private final CommandXboxController driver;
    private final CommandXboxController coDriver;
    private final Orchestra orchestra;

    public Robot() {
        PhoenixUtil.disableDaemons();
        orchestra = new Orchestra();

        // Initialize subsystems
        lights = new Lights();
        swerve = new Swerve();
        swerve.applyOrchestra(orchestra);

        storage = new Storage();
        shooter = new Shooter();
        intake = new Intake();

        matchData = new MatchData();

        // Initialize compositions
        routines = new Routines(this);
        autos = new Autos(this);

        // Initialize controllers
        driver = new CommandXboxController(Constants.DRIVER);
        coDriver = new CommandXboxController(Constants.CO_DRIVER);

        // Set default commands
        swerve.setDefaultCommand(swerve.drive(this::driverX, this::driverY, this::driverAngular));

        // Create triggers
        // Trigger allowGoosing = coDriver.a().negate();
        // Trigger changedReference = RobotModeTriggers.teleop().and(swerve::changedReference);

        // Driver bindings
        driver.leftTrigger().onTrue(swerve.tareRotation());

        // driver.povLeft().onTrue(swerve.tareRotation());
        // driver.y().whileTrue(swerve.apfDrive(() -> new Pose2d(2.26, 4.39, Rotation2d.fromDegrees(0)), () -> 0.25));
        // driver.x().whileTrue(swerve.apfDrive(() -> new Pose2d(3.287, 0.607, Rotation2d.fromDegrees(0)), () -> 0.25));

        driver
            .povUp()
            .and(driver.leftBumper())
            .whileTrue(routines.driveTrench(() -> WaypointHeading.NORTH, () -> true));
        driver
            .povUp()
            .and(driver.rightBumper())
            .whileTrue(routines.driveTrench(() -> WaypointHeading.NORTH, () -> false));
        driver
            .povDown()
            .and(driver.leftBumper())
            .whileTrue(routines.driveTrench(() -> WaypointHeading.SOUTH, () -> true));
        driver
            .povDown()
            .and(driver.rightBumper())
            .whileTrue(routines.driveTrench(() -> WaypointHeading.SOUTH, () -> false));
        driver.a().whileTrue(routines.aimAtHub(() -> 0.2));
        // driver.b().whileTrue(swerve.drive(this::driverX, this::driverY, () -> swerve.getHubAngular()));
        driver.b().whileTrue(swerve.driveFacingHub(this::driverX, this::driverY, this::driverAngular));
        // driver.a().whileTrue(routines.refuelFromDepot());
        // driver.b().whileTrue(routines.refuelFromNeutral());
        // driver.a().whileTrue(swerve.apfDrive(() -> Field.WAYPOINT_GOAL_FAR.get(), () -> 12.0));
        // driver.povLeft().whileTrue(swerve.driveTrench(() -> false));
        // driver.povRight().whileTrue(swerve.driveTrench(() -> true));
        driver.rightStick().whileTrue(swerve.turboSpin(this::driverX, this::driverY, this::driverAngular));

        // driver
        //     .povRight()
        //     .whileTrue(
        //         (swerve.apfDrive(() -> swerve.getFuelPose(), () -> 0.25)).until(() -> swerve.getFuelPose() == null)
        //     );
        // driver.rightBumper().whileTrue(swerve.resetOdometry());

        // changedReference.onTrue(new RumbleCommand(driver, 1.0).withTimeout(0.2));

        // Co-driver bindings
        //coDriver.a().onTrue(none()); // Reserved (No goosing around)
        //        coDriver.x().onTrue(storage.moveMotorCommand(true));
        //        coDriver.b().onTrue(storage.moveMotorCommand(false));

        // shooter.setDefaultCommand(shooter.idleShooterCommand());
        coDriver.leftTrigger().whileTrue(routines.readyFeederShooter());

        coDriver.rightTrigger().whileTrue(shooter.readyShooter());
        coDriver.rightTrigger().and(coDriver.povRight()).whileTrue(routines.shootFuel());

        // coDriver.rightBumper().onTrue(shooter.idleShooterCommand());
        // coDriver.a().whileTrue(storage.spill());
        // coDriver.y().whileTrue(storage.shoot());
        coDriver.a().whileTrue(intake.extendIntakeTest());
        coDriver.y().whileTrue(intake.retrackIntakeTest());

        coDriver.x().whileTrue(intake.moveIntakeTest(false));
        coDriver.b().whileTrue(intake.moveIntakeTest(true));
        // coDriver.povUp().and(coDriver.rightBumper()).whileTrue(Commands.none());
        //        coDriver.x().whileTrue(routines.shootFuel());
        //        coDriver.x().whileTrue(routines.shootFuel());
        // coDriver.b().whileTrue(routines.releaseAll());
        //        coDriver.povRight().whileTrue(storage.moveMotorCommand(false));
        //        coDriver.povLeft().whileTrue(storage.moveMotorCommand(true));
        coDriver.povUp().whileTrue(storage.moveMotorCommand(true));
        coDriver.povDown().whileTrue(storage.moveMotorCommand(false));
        coDriver.rightBumper().onTrue(swerve.playMusic("enemy").ignoringDisable(true));
        //

        // Setup lights
        scheduler.schedule(routines.lightsPreMatch(autos::defaultSelected));

        RobotModeTriggers.autonomous().whileTrue(routines.selfDriveLights());
        RobotModeTriggers.disabled().whileTrue(routines.lightsDisabledMode());
        // Disable loop overrun warnings from the command
        // scheduler, since we already log loop timings
        DisableWatchdog.in(scheduler, "m_watchdog");

        // Configure the brownout threshold to match RIO 1
        RobotController.setBrownoutVoltage(6.3);

        // Enable real-time thread priority
        enableRT(true);
    }

    /**
     * Returns the current match time in seconds.
     */
    public static double matchTime() {
        return Math.max(0.0, DriverStation.getMatchTime());
    }

    @NotLogged
    public double driverX() {
        return driver.getLeftX();
    }

    @NotLogged
    public double driverY() {
        return driver.getLeftY();
    }

    @NotLogged
    public double driverAngular() {
        return -driver.getRightX();
    }

    @Override
    public void robotPeriodic() {
        Profiler.run("scheduler", scheduler::run);
        Profiler.run("lights", lights::update);

        MatchData.shouldIShoot();
        // try {
        //     SmartDashboard.putString("fuelPose", Objects.requireNonNull(swerve.getFuelPose()).toString());
        // } catch (Exception ignored) {}
    }

    private void playSong(String filename) {
        orchestra.loadMusic(filename);
        orchestra.play();
    }
}
