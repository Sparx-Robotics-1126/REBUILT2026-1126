package org.team1126.robot.subsystems;

import static org.team1126.robot.Constants.INTAKE_MOTOR;
import static org.team1126.robot.Constants.PIVOT_MOTOR;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.util.command.GRRSubsystem;

public final class Intake extends GRRSubsystem {

    private final SparkFlex intakeMotor;
    private final SparkFlex pivotMotor;

    private SparkFlexConfig intakeConfig;
    private final RelativeEncoder intakeEncoder;
    private SparkClosedLoopController intakeController;
    private final Tunables.TunableInteger intakeSpeed = tunables.value("Intake Speed", 125);
    private static final TunableTable tunables = Tunables.getNested("intake");

    private SparkFlexConfig pivotConfig;
    private final SparkAbsoluteEncoder pivotEncoder;
    private SparkClosedLoopController pivotController;
    private final Tunables.TunableDouble pivotPosition = tunables.value("Pivot Position", -.28);

    // private final SparkMax moveStorage;
    // private SparkMaxConfig moveStorageConfig;
    // private final RelativeEncoder moveStorageEncoder;
    // private SparkClosedLoopController moveStorageController;
    // private static final TunableTable moveStorageTunables = Tunables.getNested("moveStorage");

    //    private boolean isOn;
    //    private final Tunables.TunableDouble voltage;

    public Intake() {
        intakeMotor = new SparkFlex(INTAKE_MOTOR, SparkLowLevel.MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();
        intakeConfig = new SparkFlexConfig();

        pivotMotor = new SparkFlex(PIVOT_MOTOR, SparkLowLevel.MotorType.kBrushless);
        pivotEncoder = pivotMotor.getAbsoluteEncoder();
        pivotConfig = new SparkFlexConfig();
        pivotController = pivotMotor.getClosedLoopController();

        pivotConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .p(.4, ClosedLoopSlot.kSlot0)
                .i(0, ClosedLoopSlot.kSlot0)
                .d(0, ClosedLoopSlot.kSlot0)
                .feedForward.kG(.05, ClosedLoopSlot.kSlot0);

        pivotConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .p(.4, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .feedForward.kG(.05, ClosedLoopSlot.kSlot1);
//                .maxMotion.maxAcceleration(100).cruiseVelocity(100).allowedProfileError(1)

        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // moveStorage = new SparkMax(MOVE_STORAGE_MOTOR, SparkLowLevel.MotorType.kBrushless);
        // moveStorageEncoder = moveStorage.getEncoder();
        // moveStorageConfig = new SparkMaxConfig();
        // moveStorageController = moveStorage.getClosedLoopController();

        //        isOn = false;

        intakeController = intakeMotor.getClosedLoopController();
        intakeConfig
            .smartCurrentLimit(40)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .inverted(false)
            .openLoopRampRate(0.25)
            .closedLoopRampRate(0.25);

        intakeConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed
            // loop slot, as it will default to slot 0.
            .p(0.4)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            // Set PID values for velocity control in slot 1
            .feedForward
            // kV is now in Volts, so we multiply by the nominal voltage (12V)
            .kV(.05);

        intakeConfig.closedLoop.maxMotion
            // Set MAXMotion parameters for position control. We don't need to pass
            // a closed loop slot, as it will default to slot 0.
            .cruiseVelocity(85)
            .maxAcceleration(85)
            .allowedProfileError(1);

        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        //        shooterController.setSetpoint(this.shooterShootSpeed, SparkBase.ControlType.kMAXMotionVelocityControl);
        tunables.add("Intake Motor", intakeMotor);
        tunables.add("Pivot Motor", pivotMotor);
    }

    public Command moveIntakeTest(boolean reverse){
        return commandBuilder()
            .onExecute(() -> moveIntakeMotor(reverse))
                .onEnd(this::stopIntake);
    }

    public Command moveIntakeMotorCommand(boolean reverse) {
        return commandBuilder()
            .onExecute(() -> moveIntakeMotor(reverse))
            .onEnd(() -> intakeController.setSetpoint(0, SparkBase.ControlType.kMAXMotionVelocityControl));
    }

    public void moveIntakeMotor(boolean reverse) {
        if (reverse) {
            intakeController.setSetpoint(-this.intakeSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
        } else {
            intakeController.setSetpoint(this.intakeSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
        }
    }

    public Command spill() {
        //toggle(true);
        return commandBuilder()
            .onExecute(() -> moveIntakeMotor(true))
            .onEnd(this::stopIntake);
    }

    public Command intake() {
        //toggle(true);
        return commandBuilder()
            .onExecute(() -> moveIntakeMotor(false))
            .onEnd(() -> intakeMotor.set(0));
    }

    private void stopIntake() {
        intakeMotor.setVoltage(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake at set point?", intakeController.isAtSetpoint());
        SmartDashboard.putNumber("Velocity", this.intakeEncoder.getVelocity());
        SmartDashboard.putNumber("Position", this.pivotEncoder.getPosition());
        //        if (isOn) {
        //            intakeController.setSetpoint(voltage.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
        //        }
    }


    public void moveMotorPos(double position) {
        this.pivotController.setSetpoint(position, SparkBase.ControlType.kPosition);
    }

    public Command extendIntakeTest() {
        return commandBuilder()
            .onExecute(() -> this.moveMotorPos(pivotPosition.get()));
    }
    public Command extendIntake() {
        return commandBuilder()
            .onExecute(() -> this.moveMotorPos(pivotPosition.get()))
            .onEnd(interrupted -> {
                // Stop driving and let it relax wherever it ended up
                pivotMotor.setVoltage(0.0);

                // Put the pivot motor into Coast
                var coastCfg = new SparkFlexConfig().idleMode(SparkBaseConfig.IdleMode.kCoast);

                pivotMotor.configure(coastCfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            });
    }

    public Command retrackIntakeTest() {
        return commandBuilder()
            .onExecute(() -> this.moveMotorPos(0));
    }

    public Command retrackIntake() {
        return commandBuilder()
            .onExecute(() -> this.moveMotorPos(0))
            .onEnd(interrupted -> {
                // Keep actively holding home when the command ends
                pivotController.setSetpoint(0, SparkBase.ControlType.kPosition);

                // Use Brake to resist drifting away from home
                var brakeCfg = new SparkFlexConfig().idleMode(SparkBaseConfig.IdleMode.kBrake);

                pivotMotor.configure(brakeCfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            });
    }
}
