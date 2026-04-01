package org.team1126.robot.subsystems;

import static org.team1126.robot.Constants.INTAKE_MOTOR;
import static org.team1126.robot.Constants.PIVOT_MOTOR_LEAD;
import static org.team1126.robot.Constants.PIVOT_MOTOR_FOLLOW;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.util.command.GRRSubsystem;

@Logged
public final class Intake extends GRRSubsystem {

    private final TalonFX intakeMotor;
    private final SparkFlex pivotMotorLead;
    private final SparkFlex pivotMotorFollow;

    private TalonFXConfiguration intakeConfig;
    private final Tunables.TunableInteger intakeSpeed = tunables.value("Intake Speed", 500);
    private static final TunableTable tunables = Tunables.getNested("intake");

    private SparkFlexConfig pivotConfig;
    private SparkClosedLoopController pivotController;
    private final Tunables.TunableDouble pivotPosition = tunables.value("Pivot Position", -16.7);
    
    private List<Double> outputCurrent = new ArrayList<>();
    private List<Double> appliedOutput = new ArrayList<>();
    private List<Double> busVoltage = new ArrayList<>();
    private List<Double> intakeTemperature = new ArrayList<>();

    public Intake() {
        intakeMotor = new TalonFX(INTAKE_MOTOR);
        intakeConfig = new TalonFXConfiguration();

        pivotMotorLead = new SparkFlex(PIVOT_MOTOR_LEAD, SparkLowLevel.MotorType.kBrushless);
        pivotMotorFollow = new SparkFlex(PIVOT_MOTOR_FOLLOW, SparkLowLevel.MotorType.kBrushless);
        pivotConfig = new SparkFlexConfig();
        pivotController = pivotMotorLead.getClosedLoopController();

        pivotConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        pivotConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(.95, ClosedLoopSlot.kSlot0)
            .i(0, ClosedLoopSlot.kSlot0)
            .d(0, ClosedLoopSlot.kSlot0)
            .feedForward.kCos(.0, ClosedLoopSlot.kSlot0);
        pivotConfig.closedLoop.maxMotion
            .maxAcceleration(1500, ClosedLoopSlot.kSlot0) // REDUCED from 200 — slower ramp
            .allowedProfileError(1, ClosedLoopSlot.kSlot0)
            .cruiseVelocity(750, ClosedLoopSlot.kSlot0) // Changed from 1000 to match test setpoint
            .allowedProfileError(1, ClosedLoopSlot.kSlot0);

        pivotConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(.9, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .feedForward.kCos(.9, ClosedLoopSlot.kSlot1);
        pivotConfig.closedLoop.maxMotion
            .maxAcceleration(1000, ClosedLoopSlot.kSlot1) // REDUCED from 200 — slower ramp
            .allowedProfileError(1, ClosedLoopSlot.kSlot1)
            .cruiseVelocity(650, ClosedLoopSlot.kSlot1) // Changed from 1000 to match test setpoint
            .allowedProfileError(1, ClosedLoopSlot.kSlot1);

        pivotMotorLead.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        SparkFlexConfig followConfig = new SparkFlexConfig();
        // followConfig.inverted(false);
        
        followConfig.follow(PIVOT_MOTOR_LEAD,true);
        pivotMotorFollow.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Configure TalonFX intake motor
        intakeConfig.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;
        intakeConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 40;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Velocity control PID (slot 0)
        intakeConfig.Slot0.kP = 0.12;
        intakeConfig.Slot0.kI = 0.0;
        intakeConfig.Slot0.kD = 0.0;
        intakeConfig.Slot0.kV = 0.12; // velocity feedforward

        intakeMotor.getConfigurator().apply(intakeConfig);

        tunables.add("Intake Motor", intakeMotor);
        tunables.add("Pivot Motor Lead", pivotMotorLead);
         tunables.add("Pivot Motor Follower", pivotMotorFollow);
    }

    @NotLogged
    public Command moveIntake(boolean reverse) {
        return commandBuilder()
            .onExecute(() -> moveIntakeMotor(reverse))
            .onEnd(this::stopIntake);
    }

    @NotLogged
    public Command moveIntakeMotorCommand(boolean reverse) {
        return commandBuilder()
            .onExecute(() -> moveIntakeMotor(reverse))
            .onEnd(() -> intakeMotor.setControl(new VelocityVoltage(0)));
    }

    @NotLogged
    public void moveIntakeMotor(boolean reverse) {
        double speed = reverse ? -this.intakeSpeed.get() : this.intakeSpeed.get();
        intakeMotor.setControl(new VelocityVoltage(speed));
    }

    @NotLogged
    public Command spill() {
        return commandBuilder()
            .onExecute(() -> moveIntakeMotor(true))
            .onEnd(this::stopIntake);
    }

    @NotLogged
    public Command intake() {
        return commandBuilder()
            .onExecute(() -> moveIntakeMotor(false))
            .onEnd(() -> intakeMotor.set(0));
    }

    @NotLogged
    private void stopIntake() {
        intakeMotor.setVoltage(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake at set point?", intakeMotor.getClosedLoopError().getValueAsDouble() < 50);
        SmartDashboard.putNumber("Velocity", intakeMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Position", this.pivotMotorLead.getEncoder().getPosition());
        SmartDashboard.putNumber("Pivot Temp", this.pivotMotorLead.getMotorTemperature());
        outputCurrent.add(intakeMotor.getSupplyCurrent().getValueAsDouble());
        appliedOutput.add(intakeMotor.getDutyCycle().getValueAsDouble());
        busVoltage.add(intakeMotor.getSupplyVoltage().getValueAsDouble());
        intakeTemperature.add(intakeMotor.getDeviceTemp().getValueAsDouble());
        
        SmartDashboard.putNumber("Intake Motor Output Current", intakeMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake Motor Applied Output", intakeMotor.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("Intake Motor Bus Voltage", intakeMotor.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake Motor Temperature", intakeMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Motor Output Current", pivotMotorLead.getOutputCurrent());
        SmartDashboard.putNumber("Pivot Motor Applied Output", pivotMotorLead.getAppliedOutput());  
        SmartDashboard.putNumber("Pivot Motor Bus Voltage", Math.abs(pivotMotorLead.getBusVoltage()));
        SmartDashboard.putNumber("Pivot Motor Temperature", pivotMotorLead.getMotorTemperature());
    }

    @NotLogged
    public void moveMotorPosOut(double position, boolean spinIntake) {
        this.pivotController.setSetpoint(
            position,
            SparkBase.ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0
        );
    }

    @NotLogged
    public void moveMotorPosIn(double position) {
        this.pivotController.setSetpoint(
            position,
            SparkBase.ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot1
        );
    }


       @NotLogged
    public Command extendIntake() {
        return commandBuilder().onExecute(() -> this.moveMotorPosOut(pivotPosition.get(), false));
    }
    /**
     * Agitates the hopper by jostling the intake up and down while pulling balls inwards.
     */
    public Command agitate() {
        return sequence( extendIntake().withTimeout(0.4),
                        retrackIntake().withTimeout(0.4))
            .repeatedly()
            .withName("Intake.agitate()");
    }


    @NotLogged
    public Command retrackIntake() {
        return commandBuilder().onExecute(() -> this.moveMotorPosIn(1));
    }

    // @NotLogged
    // public Command retrackIntake() {
    //     return commandBuilder()
    //         .onExecute(() -> this.moveMotorPosIn(0))
    //         .onEnd(interrupted -> {
    //             // Keep actively holding home when the command ends
    //             pivotController.setSetpoint(0, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);

    //             // Use Brake to resist drifting away from home
    //             var brakeCfg = new SparkFlexConfig().idleMode(SparkBaseConfig.IdleMode.kBrake);

    //             pivotMotorLead.configure(brakeCfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    //         });
    // }

    @NotLogged
    private void stopPivot() {
        pivotMotorLead.setVoltage(0);
    }

    public List<Double> getOutputCurrent() {
        return outputCurrent;
    }
    
    public List<Double> getAppliedOutput() {
        return appliedOutput;
        
    }

    public List<Double> getBusVoltage() {
        return busVoltage;
    }
    
    public List<Double> getIntakeTemperature() {
        return intakeTemperature;
    }
}
