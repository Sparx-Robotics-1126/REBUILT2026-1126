package org.team1126.robot.subsystems;

import static org.team1126.robot.Constants.FEEDER_MOTOR;
import static org.team1126.robot.Constants.SHOOTER_MOTOR;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.util.command.GRRSubsystem;

public final class Shooter extends GRRSubsystem {

    private static final TunableTable tunables = Tunables.getNested("shooter");
    private static final Tunables.TunableDouble shooterKV = tunables.value("Shooter kV", 0.00001);

    private final SparkMax feederMotor;
    private final SparkMaxConfig feederConfig;
    private final RelativeEncoder feederEncoder;
    // private final SparkAbsoluteEncoder feederAbsoluteEncoder;
    private final Tunables.TunableInteger feederSpeed = tunables.value("Feeder Speed", 50);
    private final SparkClosedLoopController feederController;

    private final SparkFlex shooterMotor;
    private final SparkFlexConfig shooterConfig;
    private final RelativeEncoder shooterEncoder;
    private final SparkAbsoluteEncoder shooterAbsoluteEncoder;
    private final SparkClosedLoopController shooterController;
    private final Tunables.TunableInteger shooterShootSpeed = tunables.value("Shoot Speed", 100);
    private final Tunables.TunableDouble shooterIdleSpeed = tunables.value("Idle Speed", 0.0);

    private ShooterStates state = ShooterStates.kIdle;

    public static enum ShooterStates {
        kIdle,
        kShooting
    }

    public Shooter() {
        // feederMotor = new SparkMax(FEEDER_MOTOR, SparkLowLevel.MotorType.kBrushless);

        // feederConfig = new SparkMaxConfig();

        // feederConfig
        //     .smartCurrentLimit(40)
        //     .idleMode(SparkBaseConfig.IdleMode.kBrake)
        //     .inverted(true)
        //     .openLoopRampRate(0.25)
        //     .closedLoopRampRate(0.25);

        // feederEncoder = feederMotor.getEncoder();

        // feederAbsoluteEncoder = feederMotor.getAbsoluteEncoder();

        // feederController = feederMotor.getClosedLoopController();

        // feederSpeed = tunables.value("Feeder Speed", 800);
        // feederConfig.closedLoop
        //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //     // Set PID values for position control. We don't need to pass a closed
        //     // loop slot, as it will default to slot 0.
        //     .p(0.0001)
        //     .i(0.00001)
        //     .d(0)
        //     .outputRange(-1, 1)
        //     // Set PID values for velocity control in slot 1

        //     // .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
        //     .feedForward
        //     // kV is now in Volts, so we multiply by the nominal voltage (12V)
        //     .kV(0.005);
        // // .kV(12.0 / 5767);

        // feederConfig.closedLoop.maxMotion
        //     // Set MAXMotion parameters for position control. We don't need to pass
        //     // a closed loop slot, as it will default to slot 0.
        //     // .cruiseVelocity(1000)
        //     .maxAcceleration(600)
        //     .allowedProfileError(1)
        //     // Set MAXMotion parameters for velocity control in slot 1
        //     .cruiseVelocity(2000)
        //     .allowedProfileError(1);
        // feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // tunables.add("Feeder Motor", feederMotor);

        // shooterMotor = new SparkFlex(SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);

        // shooterConfig = new SparkFlexConfig();
        // shooterConfig
        //     .smartCurrentLimit(40)
        //     .idleMode(SparkBaseConfig.IdleMode.kBrake)
        //     .inverted(true)
        //     .openLoopRampRate(0.25)
        //     .closedLoopRampRate(0.25)
        //     .encoder.positionConversionFactor(1)
        //     .velocityConversionFactor(1);

        // tunables.value("Idle Speed", 0.0);

        // shooterEncoder = shooterMotor.getEncoder();

        // shooterAbsoluteEncoder = shooterMotor.getAbsoluteEncoder();
        // shooterController = shooterMotor.getClosedLoopController();

        // shooterConfig.closedLoop
        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // .p(0.00001).i(0).d(0).outputRange(-1, 1);

        // // shooterConfig.closedLoop.feedForward
        // //     .kV(.000001) // REDUCED from 0.00005 — 5x lower
        // //     .kS(0);

        // shooterConfig.closedLoop.maxMotion

        //     .maxAcceleration(100) // REDUCED from 200 — slower ramp
        //     .allowedProfileError(100)
        //     .cruiseVelocity(500) // Changed from 1000 to match test setpoint
        //     .allowedProfileError(100);

        // shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // //        shooterController.setSetpoint(this.shooterShootSpeed, SparkBase.ControlType.kMAXMotionVelocityControl);
        // tunables.add("Shooter Motor", shooterMotor);

        shooterMotor = new SparkFlex(SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        feederMotor = new SparkMax(FEEDER_MOTOR, SparkLowLevel.MotorType.kBrushless);

        shooterController = shooterMotor.getClosedLoopController();
        feederController = feederMotor.getClosedLoopController();

        shooterEncoder = shooterMotor.getEncoder();
        shooterAbsoluteEncoder = shooterMotor.getAbsoluteEncoder();
        feederEncoder = feederMotor.getEncoder();

        shooterConfig = new SparkFlexConfig();
        feederConfig = new SparkMaxConfig();

        shooterConfig
            .inverted(true)
            .closedLoop.p(0)
            .i(0)
            .d(0)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .feedForward.kV(.05);
        feederConfig.closedLoop.p(0).i(0).d(0).feedbackSensor(FeedbackSensor.kPrimaryEncoder).feedForward.kV(.05);

        shooterConfig.closedLoop.maxMotion
            .maxAcceleration(50)
            .allowedProfileError(10)
            .cruiseVelocity(100)
            .allowedProfileError(10);

        feederConfig
            .inverted(true)
            .closedLoop.maxMotion.maxAcceleration(50)
            .allowedProfileError(50)
            .cruiseVelocity(2000)
            .allowedProfileError(100);

        shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        feederMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        tunables.add("Shooter Motor", shooterMotor);
        tunables.add("Feeder Motor", feederMotor);
        tunables.add("Shooter Encoder", shooterEncoder);
        tunables.add("Shooter Absolute Encoder", shooterAbsoluteEncoder);
    }

    @Override
    public void periodic() {
        // if (state == ShooterStates.kIdle) {
        //     shooterController.setSetpoint(this.shooterIdleSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
        // } else {
        //     shooterController.setSetpoint(
        //         this.shooterShootSpeed.get(),
        //         SparkBase.ControlType.kMAXMotionVelocityControl
        //     );
        // // }

        SmartDashboard.putNumber("Velocity", shooterEncoder.getVelocity());
        SmartDashboard.putNumber("Absolute Velocity", shooterAbsoluteEncoder.getVelocity());
        // SmartDashboard.putNumber("Velocity", shooterEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter Setpoint", shooterShootSpeed.get());
        // SmartDashboard.putNumber("Feeder Velocity", feederEncoder.getVelocity());
        // SmartDashboard.putNumber("Feeder Setpoint", feederSpeed.get());
        SmartDashboard.putBoolean("Is at speed?", shooterController.isAtSetpoint());

        SmartDashboard.putNumber("Shooter kV", shooterKV.get());
    }

    private void idleShooter() {
        this.shooterController.setSetpoint(
            this.shooterIdleSpeed.get(),
            SparkBase.ControlType.kMAXMotionVelocityControl
        );
    }

    private void stopShooter() {
        this.shooterMotor.set(0);
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
        return commandBuilder().onExecute(() -> idleShooter());
        // .onEnd(() -> stopShooter());
        // return commandBuilder()
        //     .onExecute(() -> setState(ShooterStates.kIdle))
        //     .onEnd(() -> setState(ShooterStates.kIdle));
        //        setState(ShooterStates.kIdle);
        //        this.shooterShootSpeed.set(this.shooterIdleSpeed.get());
    }

    public Command readyShooter() {
        return commandBuilder()
            .onInitialize(
                () ->
                    this.shooterController.setSetpoint(
                        this.shooterShootSpeed.get(),
                        ControlType.kMAXMotionVelocityControl
                    ) // Changed from onExecute to onInitialize
            )
            .onEnd(() -> this.shooterController.setSetpoint(0, SparkBase.ControlType.kMAXMotionVelocityControl));
        // .until(() -> this.shooterController.isAtSetpoint()); // Add until condition
    }

    /**
     * Finds out if the shooter has reached enough speed to shoot
     * @return Whether or not the shooter is ready to shoot
     */
    public boolean isReady() {
        return this.shooterController.isAtSetpoint();
    }

    private void setFeederSpeed() {
        shooterController.setSetpoint(this.shooterShootSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
        feederController.setSetpoint(this.feederSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
    }

    private void stopFeeder() {
        feederController.setSetpoint(0, SparkBase.ControlType.kMAXMotionVelocityControl);
        // feederMotor.setVoltage(0);
    }

    public Command feederStop() {
        return commandBuilder().onExecute(this::stopFeeder);
    }

    public Command feedShooter() {
        return commandBuilder().onExecute(this::setFeederSpeed).onEnd(this::stopFeeder);
    }
}
