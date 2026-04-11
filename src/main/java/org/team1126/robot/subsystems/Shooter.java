package org.team1126.robot.subsystems;

import static org.team1126.robot.util.ShootParams.shooterVelocityMap;

import static org.team1126.robot.Constants.SHOOTER_MOTOR_LEAD;
import static org.team1126.robot.Constants.SHOOTER_MOTOR_FOLLOW;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.tunable.Tunables.TunableDouble;
import org.team1126.lib.util.command.GRRSubsystem;
import org.team1126.lib.util.vendors.PhoenixUtil;

public final class Shooter extends GRRSubsystem {

    private static final TunableTable tunables = Tunables.getNested("shooter");

    private final TalonFX motorLead;
    private final TalonFX motorFollow;
    private final StatusSignal<Double> closedLoopError;
    private static final TunableDouble atVelocityEpsilon = tunables.value("atVelocityEpsilon", 4.0);
    private final VelocityVoltage velocityControl;
    private final Follower motorFollowControl;

   
    private final Tunables.TunableInteger shooterShootSpeed = tunables.value("Shoot Speed", 60);
    // private final Tunables.TunableInteger shooterShootFieldSpeed = tunables.value("Shoot Field Speed", 250);
    // private final Tunables.TunableInteger shooterUnJamSpeed = tunables.value("Shooter UnJam Speed", 100);
    // private final Tunables.TunableInteger shooterIdleSpeed = tunables.value("Idle Speed", 198);
    // private final Tunables.TunableInteger shooterVelocitySetpoint = tunables.value("Shooter Setpoint", 2830);

    private boolean shootingField = false;

    public static enum ShooterStates {
        kIdle,
        kShooting
    }

    public Shooter() {


        this.motorLead = new TalonFX(SHOOTER_MOTOR_LEAD);
        this.motorFollow = new TalonFX(SHOOTER_MOTOR_FOLLOW);
      
        configureMotors();
        closedLoopError = motorLead.getClosedLoopError();

        velocityControl = new VelocityVoltage(0.0);
        velocityControl.EnableFOC = true;
        velocityControl.UpdateFreqHz = 0.0;

        motorFollowControl = new Follower(motorLead.getDeviceID(), MotorAlignmentValue.Opposed);

        tunables.add("motors", motorLead);
        tunables.add("motors", motorFollow);
        
    }

    @Override
    public void periodic() {

         BaseStatusSignal.refreshAll(closedLoopError);

        motorFollow.setControl(motorFollowControl);

        SmartDashboard.putNumber("Shooter Velocity", motorLead.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("Shooter is at speed?", shooterIsReady());
        SmartDashboard.putBoolean("Shooting Field?", this.shootingField);
        SmartDashboard.putBoolean("Shooter is at setpoint?", shooterIsReady());
        SmartDashboard.putNumber("Shooter closed loop error", closedLoopError.getValueAsDouble());
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
                velocityControl.withVelocity(velocity.getAsDouble());
                motorLead.setControl(velocityControl);
            })
            .onEnd(() -> {
                motorLead.stopMotor();
            });
    }

    // private void idleShooter() {
    //     this.shooterController.setSetpoint(
    //         this.shooterIdleSpeed.get(),
    //         SparkBase.ControlType.kMAXMotionVelocityControl
    //     );
    // }

    // public Command haltShooter() {
    //     return commandBuilder().onExecute(this::stopShooter);
    // }

    private void stopShooter() {
        this.motorLead.stopMotor();
    }

    // public Command idleShooterCommand() {
    //     return commandBuilder().onExecute(this::idleShooter);
    //     // .onEnd(() -> stopShooter());
    //     // return commandBuilder()
    //     //     .onExecute(() -> setState(ShooterStates.kIdle))
    //     //     .onEnd(() -> setState(ShooterStates.kIdle));
    //     //        setState(ShooterStates.kIdle);
    //     //        this.shooterShootSpeed.set(this.shooterIdleSpeed.get());
    // }

    public Command readyShooter() {
        this.shootingField = false;
        return commandBuilder().onExecute(() -> getReady())
        .onEnd(this::stopShooter);
    }

    // public Command readyFieldShooter(BooleanSupplier fieldShooting) {
    //     this.shootingField = true;
    //     return commandBuilder().onExecute(() -> getFieldReady(fieldShooting));
    // }

    // public void getFieldReady(BooleanSupplier fieldShooting) {
    //     shooterController.setSetpoint(
    //         this.shooterShootFieldSpeed.get(),
    //         SparkBase.ControlType.kMAXMotionVelocityControl
    //     );
    //     // feederController.setSetpoint(this.feederSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
    // }

    public void getReady() {
           motorLead.setControl(new VelocityVoltage(this.shooterShootSpeed.get()));
        // if (this.shooterIsReady()) {
        //     feederController.setSetpoint(this.feederSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
        // }
        // shooterController.setSetpoint(this.shooterShootSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
        // feederController.setSetpoint(this.feederSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
    }

    // public Command getReadyCommand() {
    //     return commandBuilder().onExecute(() -> getReady());
    // }

    // public Command readyFeeder() {
    //     return commandBuilder().onExecute(() ->
    //         feederController.setSetpoint(this.feederSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl)
    //     );
    // }

    /**
     * Finds out if the shooter has reached enough speed to shoot
     * @return Whether or not the shooter is ready to shoot
     */
    // public boolean shooterIsReady() {

    //      return motorLead.getMotionMagicAtTarget().getValue();
    //     // if (this.shooterEncoder.getVelocity() < this.shooterVelocitySetpoint.get()+ 100 
    //     // && this.shooterEncoder.getVelocity() > this.shooterVelocitySetpoint.get() - 100) {
    //     //     return true;
    //     // }
    //     // return false;
    // }
 /**
     * shooter's closeloop error is less than allowed error.
     * @return true if less
     */
    public boolean shooterIsReady() {
        return (Math.abs(closedLoopError.getValueAsDouble()) <= atVelocityEpsilon.get());
    }

    // public boolean feederIsReady() {
    //        if (this.feederEncoder.getVelocity() < this.feederVelocitySetpoint.get()+ 100 
    //     && this.feederEncoder.getVelocity() > this.feederVelocitySetpoint.get() - 100) {
    //         return true;
    //     }
    //     return false;
    //     // return this.feederController.isAtSetpoint();
    // }

    // public Command feedShooter() {
    //     return commandBuilder().onExecute(() ->
    //         feederController.setSetpoint(this.feederSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl)
    //     );
    // }

    // private void shooting(BooleanSupplier shootingField) {
    //     // feederController.setSetpoint(this.feederSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
    //     // shooterController.setSetpoint(this.shooterShootSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
    //     // if (isReady.getAsBoolean()) {
    //     if (shootingField.getAsBoolean()) {
    //         shooterController.setSetpoint(
    //             this.shooterShootFieldSpeed.get(),
    //             SparkBase.ControlType.kMAXMotionVelocityControl
    //         );
    //     } else {
    //         shooterController.setSetpoint(
    //             this.shooterShootSpeed.get(),
    //             SparkBase.ControlType.kMAXMotionVelocityControl
    //         );
    //     }

    //     // }
    // }

    // private void stopFeeder() {
    //     // feederController.setSetpoint(0, SparkBase.ControlType.kMAXMotionVelocityControl);
    //     shooterController.setSetpoint(0, SparkBase.ControlType.kMAXMotionVelocityControl);
    //     // feederMotor.setVoltage(0);
    // }

    // public Command unJamFeeder() {
    //     return commandBuilder().onExecute(this::feederUnJam).onEnd(this::stopFeeder);
    // }

    // public Command unJamShooter() {
    //     return commandBuilder().onExecute(this::shooterUnJam).onEnd(this::stopShooter);
    // }

    // private void shooterUnJam() {
    //     shooterController.setSetpoint(-this.shooterUnJamSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
    // }

    // private void feederUnJam() {
    //     // feederController.setSetpoint(-this.feederUnJamSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
    // }

    // public Command feederStop() {
    //     return commandBuilder().onExecute(this::stopFeeder);
    // }

    // public Command shoot(BooleanSupplier isReady) {
    //     return commandBuilder()
    //         .onExecute(() -> {
    //             this.shooting(() -> false);
    //         })
    //         .onEnd(this::stopFeeder);
    // }

    // public Command shootField(BooleanSupplier isReady) {
    //     return commandBuilder()
    //         .onExecute(() -> {
    //             this.shooting(() -> true);
    //         })
    //         .onEnd(this::stopFeeder);
    // }


     private void configureMotors() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.SupplyCurrentLimit = 70.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = 0.4;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.129;
        config.Slot0.kA = 0.0;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        PhoenixUtil.run(() -> motorLead.clearStickyFaults());
        PhoenixUtil.run(() -> motorLead.getConfigurator().apply(config));

        PhoenixUtil.run(() -> motorFollow.clearStickyFaults());
        PhoenixUtil.run(() -> motorFollow.getConfigurator().apply(config));

        // Inverts the direction for the other side.
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
}
