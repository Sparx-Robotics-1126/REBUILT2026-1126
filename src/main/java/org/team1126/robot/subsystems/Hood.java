package org.team1126.robot.subsystems;

import static org.team1126.robot.Constants.HOOD_MOTOR;
import static org.team1126.robot.util.ShootParams.hoodPositionMap;

import java.util.function.DoubleSupplier;

import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.tunable.Tunables.Tunable;
import org.team1126.lib.tunable.Tunables.TunableDouble;
import org.team1126.lib.util.command.GRRSubsystem;
import org.team1126.lib.util.vendors.PhoenixUtil;
import org.team1126.robot.Constants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public final class Hood extends GRRSubsystem {

    private static final TunableTable tunables = Tunables.getNested("hood");

    private static final TunableDouble atPositionEpsilon = tunables.value("atPositionEpsilon", .3);
    private static final TunableDouble stallVelocity = tunables.value("stallVelocity", 0.5);
    private static final TunableDouble homingVelocity = tunables.value("homingVelocity", -30.0); // In rotations per second.
    private static final TunableDouble zeroZero = tunables.value("zeroZero", 1.0); // In rotations per second.
private static final TunableDouble hoodPosition = tunables.value("Hood Position", 1.0); // In rotations at the rotor (gearing not included).
    private final TalonFX motor;

    private final StatusSignal<Double> closedLoopError;
    private final StatusSignal<AngularVelocity> velocity;

    private final PositionVoltage positionVoltage;
    private final VelocityTorqueCurrentFOC velocityTorque;

    private boolean isZeroed = false;

    public Hood() {
        this.motor = new TalonFX(HOOD_MOTOR);

        configureMotor();

        closedLoopError = motor.getClosedLoopError();
        velocity = motor.getVelocity();

        PhoenixUtil.run(() -> BaseStatusSignal.setUpdateFrequencyForAll(100, closedLoopError, velocity));
        PhoenixUtil.run(() ->
            BaseStatusSignal.setUpdateFrequencyForAll(50, motor.getPosition(), motor.getClosedLoopReference())
        );
        PhoenixUtil.run(() -> ParentDevice.optimizeBusUtilizationForAll(4, motor));

        positionVoltage = new PositionVoltage(0.0);
        positionVoltage.EnableFOC = true;
        positionVoltage.UpdateFreqHz = 0.0;

        velocityTorque = new VelocityTorqueCurrentFOC(0.0);
        velocityTorque.IgnoreSoftwareLimits = true;
        velocityTorque.UpdateFreqHz = 0.0;
        velocityTorque.Slot = 1;

        tunables.add("motor", motor);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(closedLoopError, velocity);
         SmartDashboard.putNumber("Hood Position", this.motor.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Hood At Position", atPosition());
        SmartDashboard.putNumber("Hood Closed Loop Error", closedLoopError.getValueAsDouble());
    }

    /**
     * Checks if the hood is at its position within {@link Hood#closedLoopError}.
     * @return True if the hood is at the position, false if it is not.
     */
    public boolean atPosition() {
        return Math.abs(closedLoopError.getValueAsDouble()) <= atPositionEpsilon.get();
    }

    /**
     * Run the hood pivot to target a specific distance based on a preset interpolating map.
     * @param distance The distance to target in meters.
     */
    public Command targetDistance(final DoubleSupplier distance) {
        return goTo(() -> hoodPositionMap.get(distance.getAsDouble())).withName("Hood.targetDistance()");
    }

  public Command targetDistanceCommand() {
        return goTo(() -> hoodPosition.get()).withName("Hood.targetDistance()");
    }
  public Command homeCommand() {
        return goTo(() -> 0).withName("Hood.homeCommand()");
    }
    public Command zeroPositionCommand() {
        return commandBuilder("Hood.zeroPositionCommand()")
        .onExecute(() -> motor.setPosition(0));
    }

    /**
     * Moves the hood to zero.
     * @param reZero If the zeroing sequence should also be ran.
     */
    public Command goToZero(boolean reZero) {
        Command goTo = goTo(zeroZero).withName("Hood.goToZero(" + reZero + ")");
        if (reZero) goTo = goTo.beforeStarting(() -> isZeroed = false);
        return goTo;
    }

    /**
     * Internal method to target a specified position.
     * @param position The hood's position in rotations at the rotor (gearing not included).
     */
    private Command goTo(final DoubleSupplier position) {
        Debouncer debouncer = new Debouncer(0.1, DebounceType.kRising);

        return commandBuilder("Hood.goTo()")
            .onInitialize(() -> debouncer.calculate(false))
            .onExecute(() -> {
                if (!isZeroed) {
                    velocityTorque.withVelocity(homingVelocity.get());
                    motor.setControl(velocityTorque);
                    if (!debouncer.calculate(Math.abs(velocity.getValueAsDouble()) < stallVelocity.get())) return;

                    isZeroed = true;
                    motor.setPosition(0.0);
                }

                positionVoltage.withPosition(position.getAsDouble());
                PhoenixUtil.run(() -> motor.setControl(positionVoltage));
            })
            .onEnd(motor::stopMotor);
    }

    private void configureMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Normal operations
        config.Slot0.kP = 16.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.08;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.0;
        config.Slot0.kA = 0.0;

        // Zeroing the hood.
        config.Slot1.kP = 12.0;
        config.Slot1.kI = 0.0; // If this is anything other than zero, it should not be.
        config.Slot1.kD = 0.0;
        config.Slot1.kG = 0.0;
        config.Slot1.kS = 0.0;
        config.Slot1.kV = 0.0;
        config.Slot1.kA = 0.0;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 22.938;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.TorqueCurrent.PeakForwardTorqueCurrent = 10.0;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -10.0;

        PhoenixUtil.run(() -> motor.clearStickyFaults());
        PhoenixUtil.run(() -> motor.getConfigurator().apply(config));
    }
}





// public final class Hood extends GRRSubsystem {
//     private static final TunableTable tunables = Tunables.getNested("hood");
//     private static final TunableDouble stallVelocity = tunables.value("stallVelocity", 0.05);
//     private static final TunableDouble homingVelocity = tunables.value("homingVelocity", -30.0);
//     private static final TunableDouble zeroZero = tunables.value("zeroZero", 1.0); // In rotations per second
//     private static final TunableDouble positionTolerance = tunables.value("positionTolerance", 0.25); // In rotations at the rotor (gearing not included)
// public static final TunableDouble hoodPosition = tunables.value("Hood Position", -1.0);
//     private final TalonFX motor;
//    private final StatusSignal<Double> closedLoopError;
//     private final PositionTorqueCurrentFOC motorPositionControl;
//     // private final VelocityTorqueCurrentFOC motorVelocityControl;
//     private final TunableDouble motorVelocity;
//     private final TunableDouble motorPosition;
    

//     private boolean isZeroed = false;

//     public Hood() {
//         this.motor = new TalonFX(Constants.HOOD_MOTOR);

//         configureMotor();
//   closedLoopError = motor.getClosedLoopError();
//         this.motorVelocity = tunables.value("velocity", 0.254);
//         this.motorPosition = tunables.value("position", .25);

//         motorPositionControl = new PositionTorqueCurrentFOC(0.0);
//         motorPositionControl.UpdateFreqHz = 0.0;
//     }

//     /**
//      * Run the hood pivot to target a specific distance based on a preset interpolating map.
//      * @param distance The distance to target in meters.
//      */
//     public Command targetDistance(final DoubleSupplier distance) {
//         return goTo(() -> hoodPositionMap.get(distance.getAsDouble())).withName("Hood.targetDistance()");
//     }

//     public boolean atPosition() {
//         return motor.getMotionMagicAtTarget().getValue();
//         // if (this.motor.getPosition().getValueAsDouble() < this.motorPosition.getAsDouble() + positionTolerance.get() 
//         // && this.motor.getPosition().getValueAsDouble() > this.motorPosition.getAsDouble() - positionTolerance.get()) {
//         //     return true;
//         // }
//         // return false;
//     }

//     public Command zeroPositiCommand() {
//         motor.setPosition(0);
//         return commandBuilder("Hood.zero()")
//         .onExecute(() -> motor.setPosition(0));
//     }
//     /**
//      * Moves the hood to zero.
//      * @param reZero If the zeroing sequence should also be ran.
//      */
//     public Command goToZero(boolean reZero) {
//         Command goTo = goTo(zeroZero).withName("Hood.goToZero(" + reZero + ")");
//         if (reZero) goTo = goTo.beforeStarting(() -> isZeroed = false);
//         return goTo;
//     }

// public Command moveHoodCommand() {
  
//     return commandBuilder("Hood.moveHoodCommand()")
//         .onExecute(() -> {
//              var position = new MotionMagicVoltage(hoodPosition.get());

//                motor.setControl(position);
//             // motorPositionControl.withPosition(hoodPosition.getAsDouble());
//             // motor.setControl(motorPositionControl);
//         })
//         .onEnd(() -> motor.stopMotor());
// }
// public Command moveHoodHomeCommand() {
  
//     return commandBuilder("Hood.moveHoodCommand()")
//         .onExecute(() -> {
//             motorPositionControl.withPosition(0);
//             motor.setControl(motorPositionControl);
//         })
//         .onEnd(() -> motor.stopMotor());
// }

//     /**
//      * Internal method to target a specified position.
//      * @param position The hood's position in rotations at the rotor (gearing not included).
//      */
//     public Command goTo( DoubleSupplier position) {
//         Debouncer debouncer = new Debouncer(0.1, DebounceType.kRising);

//         return commandBuilder("Hood.goTo()")
//             .onInitialize(() -> debouncer.calculate(false))
//             .onExecute(() -> {
//                 if (!isZeroed) {
//                     motorPositionControl.withPosition(homingVelocity.get());
//                     motor.setControl(motorPositionControl);
//                     if (!debouncer.calculate(Math.abs(motorPositionControl.getPositionMeasure().abs(null)) < stallVelocity.get())) return;

//                     isZeroed = true;
//                     motor.setPosition(0.0);
//                 }

//                 var maxVelocity = new MotionMagicVoltage(position.getAsDouble());
//                 motor.setControl(maxVelocity);
//             })
//             .onEnd(motor::stopMotor);
//     }

//     private void configureMotor() {
//         final TalonFXConfiguration config = new TalonFXConfiguration();

//         config.CurrentLimits.StatorCurrentLimit = 80.0;
//         config.CurrentLimits.SupplyCurrentLimit = 40.0;

//         config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

//         // Normal operations
//         config.Slot0.kP = .10;
//         config.Slot0.kI = 0.0;
//         config.Slot0.kD = 0.08;
//         config.Slot0.kG = 0.0;
//         config.Slot0.kS = .1;
//         config.Slot0.kV = 0.0;
//         config.Slot0.kA = 0.0;

//         // Zeroing the hood.
//         // config.Slot1.kP = 12.0;
//         // config.Slot1.kI = 0.0; // If this is anything other than zero, it should not be.
//         // config.Slot1.kD = 0.0;
//         // config.Slot1.kG = 0.0;
//         // config.Slot1.kS = 1;
//         // config.Slot1.kV = 0.0;
//         // config.Slot1.kA = 0.0;

//         // config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 22.938;
//         // config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
//         // config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
//         // config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

//         config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

//         // config.TorqueCurrent.PeakForwardTorqueCurrent = 10.0;
//         // config.TorqueCurrent.PeakReverseTorqueCurrent = -10.0;

//         PhoenixUtil.run(() -> motor.clearStickyFaults());
//         PhoenixUtil.run(() -> motor.getConfigurator().apply(config));
//         Tunables.add("Hood Motor", motor);
//     }

//     @Override
//     public void periodic() {
//                BaseStatusSignal.refreshAll(closedLoopError);

//                SmartDashboard.putNumber("Hood Closed Loop Error", motor.getVelocity().getValueAsDouble());
//         SmartDashboard.putNumber("Hood Position", this.motor.getPosition().getValueAsDouble());
//         SmartDashboard.putBoolean("Hood At Position", atPosition());
//     }
// }
