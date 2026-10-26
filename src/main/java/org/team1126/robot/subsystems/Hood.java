package org.team1126.robot.subsystems;

import static org.team1126.robot.Constants.HOOD_MOTOR;
import static org.team1126.robot.util.ShootParams.hoodPositionMap;

import java.util.function.DoubleSupplier;

import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.tunable.Tunables.TunableDouble;
import org.team1126.lib.util.command.GRRSubsystem;
import org.team1126.lib.util.vendors.PhoenixUtil;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team1126.lib.systemcheck.FaultReporter;
import org.team1126.lib.systemcheck.SystemChecks;

@Logged
public final class Hood extends GRRSubsystem {

    /** Name used for this subsystem in the system check (SystemStatus/Hood on the dashboard). */
    public static final String SYSTEM_CHECK_NAME = "Hood";

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

        // System check: watch the motor for faults and register the hood system check
        FaultReporter.getInstance().registerHardware(SYSTEM_CHECK_NAME, "hood", motor);
        FaultReporter.getInstance().registerSystemCheck(SYSTEM_CHECK_NAME, this::systemCheck);
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
    public Command goTo(final DoubleSupplier position) {
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

    // ------------------------------------------------------------------
    // System check (see lib/systemcheck/README.md)
    // ------------------------------------------------------------------

    // Check positions are in rotations at the rotor, well inside the soft limits (0 to 22.938).
    private static final double CHECK_LOW = 1.0;
    private static final double CHECK_MID = 11.0;
    private static final double CHECK_HIGH = 20.0;
    /** How close to the target counts as "there", in rotor rotations. */
    private static final double CHECK_TOLERANCE = 0.5;
    /** How long each move gets to reach its target, in seconds. */
    private static final double CHECK_MOVE_TIMEOUT = 2.5;
    /** How long homing gets to find the hard stop, in seconds. */
    private static final double CHECK_HOME_TIMEOUT = 4.0;
    /** After moving around, the hard stop should still read as ~0. More than this means the encoder lost its place. */
    private static final double CHECK_MAX_DRIFT = 0.5;

    /**
     * Hood system check:
     * <ol>
     *   <li>Home against the bottom hard stop (the same gentle, current-limited
     *       homing the hood already uses) and zero the encoder.</li>
     *   <li>Move to low, middle and high positions using the normal
     *       {@link #goTo} command, checking each one is reached in time.</li>
     *   <li>Home again and check the hard stop still reads ~0. If not, the
     *       encoder lost its place during the moves (slipping belt or gear,
     *       skipped teeth, or a loose hard stop).</li>
     * </ol>
     */
    public Command systemCheck() {
        return Commands.sequence(
            checkHome(false),
            checkMoveTo(CHECK_LOW),
            checkMoveTo(CHECK_MID),
            checkMoveTo(CHECK_HIGH),
            checkMoveTo(CHECK_LOW),
            checkHome(true)
        )
            .finallyDo(motor::stopMotor)
            .withName("Hood.systemCheck()");
    }

    private Command checkMoveTo(double target) {
        return SystemChecks.moveToPosition(
            SYSTEM_CHECK_NAME,
            "hood",
            goTo(() -> target),
            () -> motor.getPosition().getValueAsDouble(),
            target,
            CHECK_TOLERANCE,
            CHECK_MOVE_TIMEOUT,
            "rot"
        );
    }

    /**
     * Drives gently down into the bottom hard stop until the hood stalls, then zeros the encoder.
     * @param checkDrift If true, report how far the encoder reading at the hard stop is from 0.
     */
    private Command checkHome(boolean checkDrift) {
        Debouncer stalled = new Debouncer(0.1, DebounceType.kRising);
        boolean[] foundStop = { false };

        Command home = commandBuilder("Hood.systemCheck(home)")
            .onInitialize(() -> {
                stalled.calculate(false);
                foundStop[0] = false;
            })
            .onExecute(() -> {
                velocityTorque.withVelocity(homingVelocity.get());
                motor.setControl(velocityTorque);
                foundStop[0] = stalled.calculate(Math.abs(velocity.getValueAsDouble()) < stallVelocity.get());
            })
            .isFinished(() -> foundStop[0])
            .onEnd(motor::stopMotor);

        Command record = Commands.runOnce(() -> {
            if (!foundStop[0]) {
                FaultReporter.getInstance()
                    .addFault(
                        SYSTEM_CHECK_NAME,
                        String.format(
                            "[hood] never reached the bottom hard stop in %.1f s - check the motor and that nothing is blocking the hood",
                            CHECK_HOME_TIMEOUT
                        )
                    );
                return;
            }

            double positionAtStop = motor.getPosition().getValueAsDouble();
            if (checkDrift && Math.abs(positionAtStop) > CHECK_MAX_DRIFT) {
                FaultReporter.getInstance()
                    .addFault(
                        SYSTEM_CHECK_NAME,
                        String.format(
                            "[hood] hard stop read %.2f rot instead of 0 after moving - belt/gears may be slipping or the hard stop moved",
                            positionAtStop
                        ),
                        true
                    );
            }
            motor.setPosition(0.0);
            isZeroed = true;
        });

        return Commands.sequence(home.withTimeout(CHECK_HOME_TIMEOUT), record);
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
