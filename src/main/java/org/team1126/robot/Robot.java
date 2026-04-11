package org.team1126.robot;

import com.ctre.phoenix6.Orchestra;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.wpilibj2.command.Commands.none;

import org.team1126.lib.logging.LoggedRobot;
import org.team1126.lib.logging.Profiler;
import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.util.DisableWatchdog;
import org.team1126.lib.util.command.RumbleCommand;
import org.team1126.lib.util.vendors.PhoenixUtil;
import org.team1126.robot.commands.Autos;
import org.team1126.robot.commands.Routines;
import org.team1126.robot.subsystems.Feeder;
import org.team1126.robot.subsystems.Hood;
import org.team1126.robot.subsystems.Intake;
import org.team1126.robot.subsystems.Lights;
import org.team1126.robot.subsystems.Shooter;
import org.team1126.robot.subsystems.Storage;
import org.team1126.robot.subsystems.Swerve;
import org.team1126.robot.util.ShiftTracker;
import org.team1126.robot.util.nav.TrenchNavigator;

@Logged
public final class Robot extends LoggedRobot {

    private final CommandScheduler scheduler = CommandScheduler.getInstance();
  private static final TunableTable tunables = Tunables.getNested("Robot");
    public final Lights lights;
    public final Swerve swerve;
    public final Storage storage;
    public final Shooter shooter;
    public final Hood hood;
    public final Intake intake;
    public final Feeder feeder;
    public final ShiftTracker shiftTracker;
    private final boolean rumbleOn = true;

    public final Routines routines;
    public final Autos autos;

    private final CommandXboxController driver;
    private final CommandXboxController coDriver;
    private final Orchestra orchestra;
    private Command autoSelected;
    private Tunables.TunableDouble driverDefaultSpeed = tunables.value("Default Drive Speed", .62);
      private Tunables.TunableDouble driverAfterburnerSpeed = tunables.value("Afterburner Drive Speed", 1.0);
    

    public Robot() {
        PhoenixUtil.disableDaemons();
        orchestra = new Orchestra();

        // Initialize subsystems
        lights = new Lights();
        swerve = new Swerve();
        swerve.applyOrchestra(orchestra);

        TrenchNavigator.init(swerve);

        storage = new Storage();
        shooter = new Shooter();
        hood = new Hood();
        intake = new Intake();
        feeder = new Feeder();

        shiftTracker = new ShiftTracker();

        // Initialize compositions
        // Switching routines to be singleton for better access in autos.
        Routines.init(this);
        routines = Routines.getInstance();
        autos = new Autos(this);

        // Initialize controllers
        driver = new CommandXboxController(Constants.DRIVER);
        coDriver = new CommandXboxController(Constants.CO_DRIVER);

        // Set default commands
        swerve.setDefaultCommand(
            swerve.drive(this::driverX, this::driverY, this::driverAngular, Constants.DO_BEACHING_DEFAULT)
        );

        lights.sides.setDefaultCommand(routines.lightsTeleopMode());
        // Create triggers

        // Driver bindings
        driver.leftTrigger().onTrue(swerve.tareRotation());

        // driver
        //     .povUp()
        //     .and(driver.leftBumper())
        //     .whileTrue(routines.driveTrenchWithLights(() -> WaypointHeading.NORTH, () -> true));
        // driver
        //     .povUp()
        //     .and(driver.rightBumper())
        //     .whileTrue(routines.driveTrenchWithLights(() -> WaypointHeading.NORTH, () -> false));
        // driver
        //     .povDown()
        //     .and(driver.leftBumper())
        //     .whileTrue(routines.driveTrenchWithLights(() -> WaypointHeading.SOUTH, () -> true));
        // driver
        //     .povDown()
        //     .and(driver.rightBumper())
        //     .whileTrue(routines.driveTrenchWithLights(() -> WaypointHeading.SOUTH, () -> false));
        driver.a().whileTrue(swerve.driveToShootingArc(() -> 0.8));
        driver.b().whileTrue(swerve.driveFacingTarget(this::driverX, this::driverY, this::driverAngular));
        // driver.y().whileTrue(swerve.driveFacingZone(this::driverX, this::driverY, this::driverAngular));
        driver.rightTrigger().whileTrue(swerve.drive(this::driverX, this::driverY, this::driverAngular, false));
        driver.rightStick().whileTrue(swerve.turboSpin(this::driverX, this::driverY, this::driverAngular));
        driver.start().onTrue(swerve.adjustShootingRadius());

        // driver.rightBumper().whileTrue(swerve.resetOdometry());

        // changedReference.onTrue(new RumbleCommand(driver, 1.0).withTimeout(0.2));

        // Co-driver bindings
        //        coDriver.x().onTrue(storage.moveMotorCommand(true));
        //        coDriver.b().onTrue(storage.moveMotorCommand(false));

        // shooter.setDefaultCommand(shooter.idleShooterCommand());
        // coDriver.leftTrigger().whileTrue(routines.readyFeederShooter());

        // coDriver.leftTrigger().whileTrue(routines.unJamFeederShooter());
        // coDriver.rightBumper().toggleOnTrue(shooter.readyShooter());
        // coDriver.rightTrigger().toggleOnTrue(shooter.readyFieldShooter(() -> true));

        var shoot = coDriver.rightTrigger();
        // Operator shoots with right trigger, runs intake at the same time with x, forces shooting with left trigger
        shoot.whileTrue(routines.shoot(coDriver.b(), coDriver.leftTrigger()));
        coDriver.leftTrigger().onTrue(none()); // Reserved for shooting override
        coDriver.back().whileTrue(hood.zeroPositionCommand());
        coDriver.rightBumper().whileTrue(routines.staticShoot());

        // coDriver.leftTrigger().whileTrue(routines.shootFieldFuel());
        coDriver.leftBumper().toggleOnTrue(routines.shootFuelTest());
        // coDriver.povRight().whileTrue(routines.shootFuel());
        coDriver.povLeft().whileTrue(routines.shootFuelReverseStorage());
        // coDriver.rightTrigger().and(coDriver.povRight()).whileTrue(routines.shootFuel());
        // coDriver.rightTrigger().and(coDriver.povLeft()).whileTrue(routines.shootFuelReverseStorage());
        // coDriver.rightBumper().onTrue(shooter.idleShooterCommand());
        // coDriver.a().whileTrue(storage.spill());
        // coDriver.y().whileTrue(storage.shoot());
        // coDriver.povRight().whileTrue(feeder.feedShooter());
        coDriver.povLeft().whileTrue(shooter.readyShooter());
        coDriver.a().onTrue(intake.extendIntake());
        coDriver.y().whileTrue(intake.retrackIntake());

        coDriver.b().and(shoot.negate()).whileTrue(intake.moveIntake(true));
        // coDriver.b().whileTrue(intake.moveIntake(true));
        // coDriver.povUp().and(coDriver.rightBumper()).whileTrue(Commands.none());
        //        coDriver.x().whileTrue(routines.shootFuel());
        //        coDriver.x().whileTrue(routines.shootFuel());
        // coDriver.b().whileTrue(routines.releaseAll());
        //        coDriver.povRight().whileTrue(storage.moveMotorCommand(false));
        //        coDriver.povLeft().whileTrue(storage.moveMotorCommand(true));
        // coDriver.povUp().whileTrue(storage.moveMotorCommand(true));
        // coDriver.povDown().whileTrue(storage.moveMotorCommand(false));
// coDriver.b().whileTrue(hood.zeroPositiCommand().ignoringDisable(true));
        coDriver.povUp().whileTrue(hood.targetDistanceCommand(       ));
        coDriver.povDown().whileTrue(hood.goToZero(true ));
        // coDriver.rightBumper().onTrue(swerve.playMusic("enemy").ignoringDisable(true));
        //
        // RobotModeTriggers.autonomous().whileTrue(autos.outpost());
        autoSelected = autos.getAutonomousCommand();

        // Setup lights
        // scheduler.schedule(routines.lightsPreMatch(autos.runSelectedAuto()));
        // RobotModeTriggers.autonomous().whileTrue(routines.selfDriveLights());
        RobotModeTriggers.disabled().whileTrue(routines.lightsDisabledMode());

        if (rumbleOn) {
            new Trigger(() -> shiftTracker.shiftTimeLeft() < 5.0)
                .onTrue(
                    new RumbleCommand(driver, 1.0)
                        .withTimeout(0.3)
                        .onlyIf(this::isTeleop)
                        .alongWith(new RumbleCommand(coDriver, 1.0).withTimeout(0.3).onlyIf(this::isTeleop))
                )
                .onFalse(
                    new RumbleCommand(driver, 1.0)
                        .withTimeout(0.6)
                        .onlyIf(this::isTeleop)
                        .alongWith(new RumbleCommand(coDriver, 1.0).withTimeout(0.6).onlyIf(this::isTeleop))
                );
        }

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
        var speed = driver.leftBumper().getAsBoolean() ? driverAfterburnerSpeed.get() : driverDefaultSpeed.get();
        return driver.getLeftX() * speed;
        // return driver.getLeftX();
    }

    @NotLogged
    public double driverY() {
        var speed = driver.leftBumper().getAsBoolean() ? driverAfterburnerSpeed.get() : driverDefaultSpeed.get();
        return driver.getLeftY() * speed;
        // return driver.getLeftY();
    }

    @NotLogged
    public double driverAngular() {
        return -driver.getRightX();
    }

    @Override
    public void robotPeriodic() {
        Profiler.run("scheduler", scheduler::run);
        Profiler.run("lights", lights::update);

        // try {
        //     SmartDashboard.putString("fuelPose", Objects.requireNonNull(swerve.getFuelPose()).toString());
        // } catch (Exception ignored) {}
    }

    // private void playSong(String filename) {
    //     orchestra.loadMusic(filename);
    //     orchestra.play();
    // }

    @Override
    public void autonomousInit() {
        autoSelected = autos.getAutonomousCommand();

        if (autoSelected != null) {
            CommandScheduler.getInstance().schedule(autoSelected);
        }
    }
}
