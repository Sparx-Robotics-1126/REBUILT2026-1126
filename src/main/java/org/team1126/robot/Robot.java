package org.team1126.robot;

import static edu.wpi.first.wpilibj.XboxController.Axis.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team1126.lib.logging.LoggedRobot;
import org.team1126.lib.logging.Profiler;
import org.team1126.lib.util.DisableWatchdog;
// import org.team1126.lib.util.command.RumbleCommand;
import org.team1126.lib.util.vendors.PhoenixUtil;
import org.team1126.robot.commands.Autos;
import org.team1126.robot.commands.Routines;
import org.team1126.robot.subsystems.Lights;
import org.team1126.robot.subsystems.Storage;
import org.team1126.robot.subsystems.Swerve;
import org.team1126.robot.util.MatchData;
import org.team1126.robot.util.ReefSelection;

@Logged
public final class Robot extends LoggedRobot {

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    public final Lights lights;
    public final Swerve swerve;
    public final Storage storage;

    public final MatchData matchData;

    public final ReefSelection selection;

    public final Routines routines;
    public final Autos autos;

    private final CommandXboxController driver;
    private final CommandXboxController coDriver;

    public Robot() {
        PhoenixUtil.disableDaemons();

        // Initialize subsystems
        lights = new Lights();
        swerve = new Swerve();
        storage = new Storage();

        matchData = new MatchData();

        // Initialize helpers
        selection = new ReefSelection();

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
        // Trigger poo = (driver.leftBumper().or(driver.rightBumper()).negate()).and(selection::isL1);

        // Driver bindings
        driver.axisLessThan(kRightY.value, -0.5).onTrue(selection.incrementLevel());
        driver.axisGreaterThan(kRightY.value, 0.5).onTrue(selection.decrementLevel());
        driver.leftTrigger().onTrue(swerve.tareRotation());

        // driver.povLeft().onTrue(swerve.tareRotation());
        driver.y().whileTrue(swerve.apfDrive(() -> new Pose2d(2.26, 4.39, Rotation2d.fromDegrees(0)), () -> 0.25));
        driver.x().whileTrue(swerve.apfDrive(() -> new Pose2d(3.287, 0.607, Rotation2d.fromDegrees(0)), () -> 0.25));

        // driver.b().whileTrue(swerve.apfDrive(() -> new Pose2d(6.844, 0.693, Rotation2d.fromDegrees(180)), () -> 0.3));
        driver.a().whileTrue(routines.refuelFromDepot());
        driver.b().whileTrue(routines.refuelFromNeutral());
        driver.povLeft().whileTrue(swerve.apfDrive(selection::isLeft, () -> true, selection::isL4));
        driver.leftStick().whileTrue(swerve.turboSpin(this::driverX, this::driverY, this::driverAngular));

        driver
            .povRight()
            .whileTrue(
                (swerve.apfDrive(() -> swerve.getFuelPose(), () -> 0.25)).until(() -> swerve.getFuelPose() == null)
            );
        driver.rightTrigger().whileTrue(swerve.resetOdometry());

        // changedReference.onTrue(new RumbleCommand(driver, 1.0).withTimeout(0.2));

        // Co-driver bindings
        coDriver.a().onTrue(none()); // Reserved (No goosing around)

        coDriver.povUp().onTrue(selection.incrementLevel());
        coDriver.povDown().onTrue(selection.decrementLevel());

        // Setup lights
        scheduler.schedule(routines.lightsPreMatch(autos::defaultSelected));

        RobotModeTriggers.autonomous().whileTrue(lights.sides.flames(false));

        lights.sides.setDefaultCommand(lights.sides.levelSelection(selection));

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
        // return driver.getLeftTriggerAxis() - driver.getRightTriggerAxis();
    }

    @Override
    public void robotPeriodic() {
        Profiler.run("scheduler", scheduler::run);
        Profiler.run("lights", lights::update);

        MatchData.shouldIShoot();
        try {
            SmartDashboard.putString("fuelPose", swerve.getFuelPose().toString());
        } catch(Exception e) { }
    }
}
