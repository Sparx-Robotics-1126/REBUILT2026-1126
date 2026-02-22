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

import java.util.function.BooleanSupplier;

public final class Shooter extends GRRSubsystem {

    private static final TunableTable tunables = Tunables.getNested("shooter");
    private static final Tunables.TunableDouble shooterKV = tunables.value("Shooter kV", 0.00001);

    private final SparkMax feederMotor;
    private final SparkMaxConfig feederConfig;
    private final RelativeEncoder feederEncoder;
    // private final SparkAbsoluteEncoder feederAbsoluteEncoder;
    private final Tunables.TunableInteger feederSpeed = tunables.value("Feeder Speed", 125);
    private final SparkClosedLoopController feederController;

    private final SparkFlex shooterMotor;
    private final SparkFlexConfig shooterConfig;
    private final RelativeEncoder shooterEncoder;
    private final SparkAbsoluteEncoder shooterAbsoluteEncoder;
    private final SparkClosedLoopController shooterController;
    private final Tunables.TunableInteger shooterShootSpeed = tunables.value("Shoot Speed", 275);
    private final Tunables.TunableInteger shooterIdleSpeed = tunables.value("Idle Speed", 175);

    private ShooterStates state = ShooterStates.kIdle;

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
                .closedLoop.p(0)
                .i(0)
                .d(0)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .feedForward.kV(.05);

        shooterConfig.closedLoop.maxMotion
                .maxAcceleration(75)
                .allowedProfileError(10)
                .cruiseVelocity(275)
                .allowedProfileError(10);

        shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        feederMotor = new SparkMax(FEEDER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        feederController = feederMotor.getClosedLoopController();
        feederEncoder = feederMotor.getEncoder();
        feederConfig = new SparkMaxConfig();
        feederConfig.closedLoop.p(0).i(0).d(0).feedbackSensor(FeedbackSensor.kPrimaryEncoder).feedForward.kV(.05);

        feederConfig
            .inverted(true)
            .closedLoop
                .maxMotion
                .maxAcceleration(125)
            .allowedProfileError(5)
            .cruiseVelocity(125)
            .allowedProfileError(5);

        feederMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

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
    }

    private void idleShooter() {
        this.shooterController.setSetpoint(
            this.shooterIdleSpeed.get(),
            SparkBase.ControlType.kMAXMotionVelocityControl
        );
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

    public Command readyShooter() {
        return commandBuilder()
            .onInitialize(
                () ->
                    this.shooterController.setSetpoint(
                        this.shooterShootSpeed.get(),
                        ControlType.kMAXMotionVelocityControl
                    ) // Changed from onExecute to onInitialize
            )
            .onEnd(this::idleShooter);
        // .until(() -> this.shooterController.isAtSetpoint()); // Add until condition
    }

    public void getReady(){
        shooterController.setSetpoint(this.shooterIdleSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
        feederController.setSetpoint(this.feederSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
    }
    public Command getReadyCommand(){
        return commandBuilder().onExecute(this::getReady);
    }
    public Command readyFeeder(){
        return commandBuilder().onExecute(      ()->  feederController.setSetpoint(this.feederSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl));
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
    private void shooting(BooleanSupplier isReady) {

        shooterController.setSetpoint(this.shooterShootSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
        if (isReady.getAsBoolean()) {
            feederController.setSetpoint(this.feederSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
        }
    }

    private void stopFeeder() {
        feederController.setSetpoint(0, SparkBase.ControlType.kMAXMotionVelocityControl);
        // feederMotor.setVoltage(0);
    }

    public Command feederStop() {
        return commandBuilder().onExecute(this::stopFeeder);
    }

    public Command shoot(BooleanSupplier isReady) {
        return commandBuilder().onExecute(() -> {
            this.shooting(isReady);
        }).onEnd(this::stopFeeder);
    }
}
