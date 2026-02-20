package org.team1126.robot.subsystems;

import static org.team1126.robot.Constants.FEEDER_MOTOR;
import static org.team1126.robot.Constants.SHOOTER_MOTOR;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.util.command.GRRSubsystem;

public final class Shooter extends GRRSubsystem {

    private static final TunableTable tunables = Tunables.getNested("shooter");

    private final SparkMax feederMotor;
    private final SparkMaxConfig feederConfig;
    private final RelativeEncoder feederEncoder;
    private final SparkAbsoluteEncoder feederAbsoluteEncoder;
    private final Tunables.TunableInteger feederSpeed;
    private final SparkClosedLoopController feederController;

    private final SparkFlex shooterMotor;
    private final SparkFlexConfig shooterConfig;
    private final RelativeEncoder shooterEncoder;
    private final SparkAbsoluteEncoder shooterAbsoluteEncoder;
    private final SparkClosedLoopController shooterController;
    private final Tunables.TunableInteger shooterShootSpeed;
    private final Tunables.TunableDouble shooterIdleSpeed;

    private ShooterStates state = ShooterStates.kIdle;

    public static enum ShooterStates {
        kIdle,
        kShooting
    }

    public Shooter() {
        feederMotor = new SparkMax(FEEDER_MOTOR, SparkLowLevel.MotorType.kBrushless);

        feederConfig = new SparkMaxConfig();

        feederConfig
            .smartCurrentLimit(40)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .inverted(true)
            .openLoopRampRate(0.25)
            .closedLoopRampRate(0.25);

        feederEncoder = feederMotor.getEncoder();

        feederAbsoluteEncoder = feederMotor.getAbsoluteEncoder();

        feederController = feederMotor.getClosedLoopController();

        feederSpeed = tunables.value("Feeder Speed", 800);
        feederConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed
            // loop slot, as it will default to slot 0.
            .p(0.0001)
            .i(0.00001)
            .d(0)
            .outputRange(-1, 1)
            // Set PID values for velocity control in slot 1

            // .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
            .feedForward
            // kV is now in Volts, so we multiply by the nominal voltage (12V)
            .kV(0.005);
        // .kV(12.0 / 5767);

        feederConfig.closedLoop.maxMotion
            // Set MAXMotion parameters for position control. We don't need to pass
            // a closed loop slot, as it will default to slot 0.
            // .cruiseVelocity(1000)
            .maxAcceleration(600)
            .allowedProfileError(1)
            // Set MAXMotion parameters for velocity control in slot 1
            .cruiseVelocity(2000)
            .allowedProfileError(1);
        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        tunables.add("Feeder Motor", feederMotor);
        tunables.add("Shooter", feederAbsoluteEncoder);

        shooterMotor = new SparkFlex(SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);

        shooterConfig = new SparkFlexConfig();
        shooterConfig
            .smartCurrentLimit(40)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .inverted(true)
            .openLoopRampRate(0.25)
            .closedLoopRampRate(0.25)
            .encoder.positionConversionFactor(1)
            .velocityConversionFactor(1 / 1.4);

        shooterShootSpeed = tunables.value("Shoot Speed", 1000);
        shooterIdleSpeed = tunables.value("Idle Speed", 0.0);

        shooterEncoder = shooterMotor.getEncoder();

        shooterAbsoluteEncoder = shooterMotor.getAbsoluteEncoder();
        shooterController = shooterMotor.getClosedLoopController();

        shooterConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed
            // loop slot, as it will default to slot 0.
            .p(0.0001)
            .i(0.0)
            .d(0.0)
            .outputRange(-1, 1)
            // Set PID values for velocity control in slot 1

            // .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
            .feedForward
            // kV is now in Volts, so we multiply by the nominal voltage (12V)
            .kV(0.00005) // INCREASE this significantly - was 0.005
            .kS(0.00);
        // .kV(12.0 / 5767);

        shooterConfig.closedLoop.maxMotion
            // Set MAXMotion parameters for position control. We don't need to pass
            // a closed loop slot, as it will default to slot 0.
            // .cruiseVelocity(1000)
            .maxAcceleration(200)
            .allowedProfileError(100)
            // Set MAXMotion parameters for velocity control in slot 1
            .cruiseVelocity(800)
            .allowedProfileError(100);

        shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        //        shooterController.setSetpoint(this.shooterShootSpeed, SparkBase.ControlType.kMAXMotionVelocityControl);
        tunables.add("Shooter Motor", shooterMotor);
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
        // }
        SmartDashboard.putNumber("Velocity", shooterEncoder.getVelocity());
        // SmartDashboard.putNumber("Velocity", shooterEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter Setpoint", shooterShootSpeed.get());
        SmartDashboard.putNumber("Feeder Velocity", feederEncoder.getVelocity());
        SmartDashboard.putNumber("Feeder Setpoint", feederSpeed.get());
        SmartDashboard.putBoolean("Is at speed?", shooterController.isAtSetpoint());
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
            .onExecute(() ->
                this.shooterController.setSetpoint(
                    this.shooterShootSpeed.get(),
                    SparkBase.ControlType.kMAXMotionVelocityControl
                )
            )
            .onEnd(() -> this.shooterController.setSetpoint(0, SparkBase.ControlType.kMAXMotionVelocityControl));
        //        setState(ShooterStates.kShooting);
        //        this.shooterShootSpeed.set(this.shooterShootSpeed.get());
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
        feederMotor.setVoltage(0);
    }

    public Command feedShooter() {
        return commandBuilder().onExecute(this::setFeederSpeed).onEnd(this::stopFeeder);
    }
}
