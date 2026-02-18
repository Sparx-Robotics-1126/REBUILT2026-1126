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
    private final Tunables.TunableDouble feederSpeed;

    private final SparkFlex shooterMotor;
    private final SparkFlexConfig shooterConfig;
    private final RelativeEncoder shooterEncoder;
    private final SparkAbsoluteEncoder shooterAbsoluteEncoder;
    private final SparkClosedLoopController shooterController;
    private final Tunables.TunableDouble shooterShootSpeed;
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
            .inverted(false)
            .openLoopRampRate(0.25)
            .closedLoopRampRate(0.25);

        feederEncoder = feederMotor.getEncoder();

        feederAbsoluteEncoder = feederMotor.getAbsoluteEncoder();

        feederSpeed = tunables.value("Feeder Speed", .2);
        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        tunables.add("Feeder Motor", feederMotor);

        shooterMotor = new SparkFlex(SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);

        shooterConfig = new SparkFlexConfig();
        shooterConfig
            .smartCurrentLimit(40)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .inverted(false)
            .openLoopRampRate(0.25)
            .closedLoopRampRate(0.25)
            .encoder.positionConversionFactor(1)
            .velocityConversionFactor(1);

        shooterShootSpeed = tunables.value("Shooter Speed", .2);
        shooterIdleSpeed = tunables.value("Shooter Idle Speed", .05);

        shooterEncoder = shooterMotor.getEncoder();

        shooterAbsoluteEncoder = shooterMotor.getAbsoluteEncoder();
        shooterController = shooterMotor.getClosedLoopController();

        shooterConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed
            // loop slot, as it will default to slot 0.
            .p(0.4)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            // Set PID values for velocity control in slot 1
            .p(0.0001, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
            .feedForward
            // kV is now in Volts, so we multiply by the nominal voltage (12V)
            .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

        shooterConfig.closedLoop.maxMotion
            // Set MAXMotion parameters for position control. We don't need to pass
            // a closed loop slot, as it will default to slot 0.
            .cruiseVelocity(1000)
            .maxAcceleration(1000)
            .allowedProfileError(1)
            // Set MAXMotion parameters for velocity control in slot 1
            .maxAcceleration(500, ClosedLoopSlot.kSlot1)
            .cruiseVelocity(6000, ClosedLoopSlot.kSlot1)
            .allowedProfileError(1, ClosedLoopSlot.kSlot1);

        shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        //        shooterController.setSetpoint(this.shooterShootSpeed, SparkBase.ControlType.kMAXMotionVelocityControl);
        tunables.add("Shooter Motor", shooterMotor);
    }

    @Override
    public void periodic() {
        if (state == ShooterStates.kIdle) {
            shooterController.setSetpoint(this.shooterIdleSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
        } else {
            shooterController.setSetpoint(
                this.shooterShootSpeed.get(),
                SparkBase.ControlType.kMAXMotionVelocityControl
            );
        }

        SmartDashboard.putNumber("Velocity", shooterEncoder.getVelocity());
        SmartDashboard.putBoolean("Is at speed?", shooterController.isAtSetpoint());
    }

    private void setState(ShooterStates state) {
        this.state = state;
        if (state == ShooterStates.kShooting) {
            this.shooterShootSpeed.set(this.shooterShootSpeed.get());
        } else {
            this.shooterShootSpeed.set(this.shooterIdleSpeed.get());
        }
    }

    public ShooterStates getState() {
        return state;
    }

    public Command readyShooter() {
        return commandBuilder()
            .onExecute(() -> setState(ShooterStates.kShooting))
            .onEnd(() -> setState(ShooterStates.kIdle));
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
        feederMotor.setVoltage(feederSpeed.get());
    }

    private void stopFeeder() {
        feederMotor.setVoltage(0);
    }

    public Command feedShooter() {
        return commandBuilder().onExecute(this::feedShooter).onEnd(this::stopFeeder);
    }
}
