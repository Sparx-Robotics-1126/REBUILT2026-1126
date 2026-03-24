package org.team1126.robot.subsystems;

import static org.team1126.robot.util.ShootParams.shooterVelocityMap;

import static org.team1126.robot.Constants.FEEDER_MOTOR;
import static org.team1126.robot.Constants.SHOOTER_MOTOR;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.util.command.GRRSubsystem;

public final class Shooter extends GRRSubsystem {

    private static final TunableTable tunables = Tunables.getNested("shooter");
    // private static final Tunables.TunableDouble shooterKV = tunables.value("Shooter kV", 0.00001);

    private final SparkMax feederMotor;
    private final SparkMaxConfig feederConfig;
    private final RelativeEncoder feederEncoder;
    // private final SparkAbsoluteEncoder feederAbsoluteEncoder;
    private final Tunables.TunableInteger feederSpeed = tunables.value("Feeder Speed", 200);
    private final Tunables.TunableInteger feederUnJamSpeed = tunables.value("Feeder UnJam Speed", 85);
    private final SparkClosedLoopController feederController;

    private final SparkFlex shooterMotor;
    private final SparkFlexConfig shooterConfig;
    private final RelativeEncoder shooterEncoder;
    private final SparkAbsoluteEncoder shooterAbsoluteEncoder;
    private final SparkClosedLoopController shooterController;
    private final Tunables.TunableInteger shooterShootSpeed = tunables.value("Shoot Speed", 198);
    private final Tunables.TunableInteger shooterShootFieldSpeed = tunables.value("Shoot Field Speed", 250);
    private final Tunables.TunableInteger shooterUnJamSpeed = tunables.value("Shooter UnJam Speed", 100);
    private final Tunables.TunableInteger shooterIdleSpeed = tunables.value("Idle Speed", 198);

    private ShooterStates state = ShooterStates.kIdle;
    private boolean shootingField = false;

    public static enum ShooterStates {
        kIdle,
        kShooting
    }

    public Shooter() {
        shooterMotor = new SparkFlex(SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);

        shooterController = shooterMotor.getClosedLoopController();
        shooterEncoder = shooterMotor.getEncoder();
        shooterAbsoluteEncoder = shooterMotor.getAbsoluteEncoder();
        shooterConfig = new SparkFlexConfig();
        shooterConfig
            .inverted(true)
            .closedLoop.p(.00015)
            .i(0)
            .d(0)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .feedForward.kV(.05);
        // shooterConfig.encoder.velocityConversionFactor(.07);

        shooterConfig.closedLoop.maxMotion
            .maxAcceleration(1375)
            .allowedProfileError(5)
            .cruiseVelocity(275)
            .allowedProfileError(5);

        shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        feederMotor = new SparkMax(FEEDER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        feederController = feederMotor.getClosedLoopController();
        feederEncoder = feederMotor.getEncoder();
        feederConfig = new SparkMaxConfig();
        feederConfig.closedLoop.p(0).i(0).d(0).feedbackSensor(FeedbackSensor.kPrimaryEncoder).feedForward.kV(.05);

        feederConfig
            .inverted(true)
            .closedLoop.maxMotion.maxAcceleration(875)
            .allowedProfileError(5)
            .cruiseVelocity(175)
            .allowedProfileError(5);

        feederMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        shootingField = false;
        tunables.add("Shooter Motor", shooterMotor);
        tunables.add("Feeder Motor", feederMotor);
        tunables.add("Shooter Encoder", shooterEncoder);
        tunables.add("Shooter Absolute Encoder", shooterAbsoluteEncoder);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Velocity", shooterEncoder.getVelocity());
        SmartDashboard.putNumber("Feeder Velocity", feederEncoder.getVelocity());
        SmartDashboard.putBoolean("Shooter is at speed?", shooterController.isAtSetpoint());
        SmartDashboard.putBoolean("Feeder is at speed?", feederController.isAtSetpoint());
        SmartDashboard.putBoolean("Shooting Field?", this.shootingField);
    }

    /**
     * Run the shooter to target a specific distance based on a preset interpolating map.
     * @param distance The distance to target in meters.
     */
    public Command targetDistance(final DoubleSupplier distance) {
        return runVelocity(() -> shooterVelocityMap.get(distance.getAsDouble())).withName("Shooters.targetDistance()");
    }

    /**
     * Internal method to run both shooters at a specified velocity.
     * @param velocity The velocity in rotations/second at the rotor (gearing not included).
     */
    private Command runVelocity(final DoubleSupplier velocity) {
        return commandBuilder("Shooter.runVelocity()")
            .onExecute(() -> {
                // TODO: Add second shooter motor
                this.shooterController.setSetpoint(velocity.getAsDouble(), SparkBase.ControlType.kMAXMotionVelocityControl);
            })
            .onEnd(() -> {
                this.shooterController.setSetpoint(0, SparkBase.ControlType.kMAXMotionVelocityControl);
            });
    }

    private void idleShooter() {
        this.shooterController.setSetpoint(
            this.shooterIdleSpeed.get(),
            SparkBase.ControlType.kMAXMotionVelocityControl
        );
    }

    public Command haltShooter() {
        return commandBuilder().onExecute(this::stopShooter);
    }

    private void stopShooter() {
        this.shooterController.setSetpoint(0, SparkBase.ControlType.kMAXMotionVelocityControl);
    }

    private void setState(ShooterStates state) {
        this.state = state;
        if (state == ShooterStates.kShooting) {
            this.shooterController.setSetpoint(
                this.shooterShootSpeed.get(),
                SparkBase.ControlType.kMAXMotionVelocityControl
            );
            // this.shooterShootSpeed.set(this.shooterShootSpeed.get());
        } else {
            this.shooterController.setSetpoint(
                this.shooterIdleSpeed.get(),
                SparkBase.ControlType.kMAXMotionVelocityControl
            );
            // this.shooterShootSpeed.set(this.shooterIdleSpeed.get());
        }
    }

    public ShooterStates getState() {
        return state;
    }

    public Command idleShooterCommand() {
        return commandBuilder().onExecute(this::idleShooter);
        // .onEnd(() -> stopShooter());
        // return commandBuilder()
        //     .onExecute(() -> setState(ShooterStates.kIdle))
        //     .onEnd(() -> setState(ShooterStates.kIdle));
        //        setState(ShooterStates.kIdle);
        //        this.shooterShootSpeed.set(this.shooterIdleSpeed.get());
    }

    private boolean getFieldShooting() {
        return this.shootingField;
    }

    public Command readyShooter() {
        this.shootingField = false;
        return commandBuilder().onExecute(() -> getReady());
    }

    public Command readyFieldShooter(BooleanSupplier fieldShooting) {
        this.shootingField = true;
        return commandBuilder().onExecute(() -> getFieldReady(fieldShooting));
    }

    public void getFieldReady(BooleanSupplier fieldShooting) {
        shooterController.setSetpoint(
            this.shooterShootFieldSpeed.get(),
            SparkBase.ControlType.kMAXMotionVelocityControl
        );
        // feederController.setSetpoint(this.feederSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
    }

    public void getReady() {
        shooterController.setSetpoint(this.shooterShootSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
        // shooterController.setSetpoint(this.shooterShootSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
        // feederController.setSetpoint(this.feederSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
    }

    public Command getReadyCommand() {
        return commandBuilder().onExecute(() -> getReady());
    }

    public Command readyFeeder() {
        return commandBuilder().onExecute(() ->
            feederController.setSetpoint(this.feederSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl)
        );
    }

    /**
     * Finds out if the shooter has reached enough speed to shoot
     * @return Whether or not the shooter is ready to shoot
     */
    public boolean shooterIsReady() {
        return this.shooterController.isAtSetpoint();
    }

    public boolean feederIsReady() {
        return this.feederController.isAtSetpoint();
    }

    public Command feedShooter() {
        return commandBuilder().onExecute(() ->
            feederController.setSetpoint(this.feederSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl)
        );
    }

    private void shooting(BooleanSupplier shootingField) {
        feederController.setSetpoint(this.feederSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
        // shooterController.setSetpoint(this.shooterShootSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
        // if (isReady.getAsBoolean()) {
        if (shootingField.getAsBoolean()) {
            shooterController.setSetpoint(
                this.shooterShootFieldSpeed.get(),
                SparkBase.ControlType.kMAXMotionVelocityControl
            );
        } else {
            shooterController.setSetpoint(
                this.shooterShootSpeed.get(),
                SparkBase.ControlType.kMAXMotionVelocityControl
            );
        }

        // }
    }

    private void stopFeeder() {
        feederController.setSetpoint(0, SparkBase.ControlType.kMAXMotionVelocityControl);
        shooterController.setSetpoint(0, SparkBase.ControlType.kMAXMotionVelocityControl);
        // feederMotor.setVoltage(0);
    }

    public Command unJamFeeder() {
        return commandBuilder().onExecute(this::feederUnJam).onEnd(this::stopFeeder);
    }

    public Command unJamShooter() {
        return commandBuilder().onExecute(this::shooterUnJam).onEnd(this::stopShooter);
    }

    private void shooterUnJam() {
        shooterController.setSetpoint(-this.shooterUnJamSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
    }

    private void feederUnJam() {
        feederController.setSetpoint(-this.feederUnJamSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
    }

    public Command feederStop() {
        return commandBuilder().onExecute(this::stopFeeder);
    }

    public Command shoot(BooleanSupplier isReady) {
        return commandBuilder()
            .onExecute(() -> {
                this.shooting(() -> false);
            })
            .onEnd(this::stopFeeder);
    }

    public Command shootField(BooleanSupplier isReady) {
        return commandBuilder()
            .onExecute(() -> {
                this.shooting(() -> true);
            })
            .onEnd(this::stopFeeder);
    }
}
