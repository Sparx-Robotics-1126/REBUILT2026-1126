package org.team1126.robot.subsystems;

import static org.team1126.robot.Constants.STORAGE_MOTOR;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.util.command.GRRSubsystem;

public final class Intake extends GRRSubsystem {

    private final SparkMax intakeMotor;
    private SparkMaxConfig config;
    private final RelativeEncoder intakeEncoder;
    private SparkClosedLoopController intakeController;
    private static final TunableTable tunables = Tunables.getNested("intake");

    //    private boolean isOn;
    private final Tunables.TunableDouble voltage;

    public Intake() {
        this.voltage = tunables.value("Voltage", .5);

        intakeMotor = new SparkMax(STORAGE_MOTOR, SparkLowLevel.MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();
        config = new SparkMaxConfig();

        //        isOn = false;

        intakeController = intakeMotor.getClosedLoopController();
        config
            .smartCurrentLimit(40)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .inverted(false)
            .openLoopRampRate(0.25)
            .closedLoopRampRate(0.25);

        config.closedLoop
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

        config.closedLoop.maxMotion
            // Set MAXMotion parameters for position control. We don't need to pass
            // a closed loop slot, as it will default to slot 0.
            .cruiseVelocity(1000)
            .maxAcceleration(1000)
            .allowedProfileError(1)
            // Set MAXMotion parameters for velocity control in slot 1
            .maxAcceleration(500, ClosedLoopSlot.kSlot1)
            .cruiseVelocity(6000, ClosedLoopSlot.kSlot1)
            .allowedProfileError(1, ClosedLoopSlot.kSlot1);

        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        //        shooterController.setSetpoint(this.shooterShootSpeed, SparkBase.ControlType.kMAXMotionVelocityControl);
        tunables.add("Intake Motor", intakeMotor);
    }

    public Command moveMotorCommand(boolean reverse) {
        return commandBuilder()
            .onExecute(() -> moveMotor(reverse))
            .onEnd(() -> intakeMotor.set(0));
    }

    public void moveMotor(boolean reverse) {
        if (reverse) {
            intakeMotor.set(-this.voltage.get());
        } else {
            intakeMotor.set(this.voltage.get());
        }
    }

    public Command spill() {
        //toggle(true);
        return commandBuilder()
            .onExecute(() -> moveMotor(true))
            .onEnd(this::stop);
    }

    public Command take() {
        //toggle(true);
        return commandBuilder()
            .onExecute(() -> moveMotor(false))
            .onEnd(() -> intakeMotor.set(0));
    }

    private void stop() {
        intakeMotor.setVoltage(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake at set point?", intakeController.isAtSetpoint());
        SmartDashboard.putNumber("Velocity", this.intakeEncoder.getVelocity());
        //        if (isOn) {
        //            intakeController.setSetpoint(voltage.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
        //        }
    }

    public void toggle(boolean mode) {
        //        isOn = mode;
    }
}
