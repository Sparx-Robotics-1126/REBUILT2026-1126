package org.team1126.robot.subsystems;

import static org.team1126.robot.Constants.INTAKE_MOTOR;
import static org.team1126.robot.Constants.PIVOT_MOTOR_LEAD;
import static org.team1126.robot.Constants.PIVOT_MOTOR_FOLLOW;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
import edu.wpi.first.wpilibj2.command.Commands;
import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.util.command.GRRSubsystem;
import org.team1126.lib.util.vendors.PhoenixUtil;
import org.team1126.lib.systemcheck.FaultReporter;
import org.team1126.lib.systemcheck.SystemChecks;

@Logged
public final class Intake extends GRRSubsystem {

    /** Name used for this subsystem in the system check (SystemStatus/Intake on the dashboard). */
    public static final String SYSTEM_CHECK_NAME = "Intake";

    private final TalonFX intakeMotor;
    private final SparkFlex pivotMotorLead;
    private final SparkFlex pivotMotorFollow;

    // private TalonFXConfiguration intakeConfig;
        private final VelocityTorqueCurrentFOC intakeVelocityControl;
    private final Tunables.TunableInteger intakeSpeed = tunables.value("Intake Speed", 200);
        private final Tunables.TunableInteger intakeAcceleration = tunables.value("Intake Acceleration", 800);
    private static final TunableTable tunables = Tunables.getNested("intake");

    private SparkFlexConfig pivotConfig;
    private SparkClosedLoopController pivotController;
    private final Tunables.TunableDouble pivotPosition = tunables.value("Pivot Position", -1.75);
    
    private List<Double> outputCurrent = new ArrayList<>();
    private List<Double> appliedOutput = new ArrayList<>();
    private List<Double> busVoltage = new ArrayList<>();
    private List<Double> intakeTemperature = new ArrayList<>();

    public Intake() {
        intakeMotor = new TalonFX(INTAKE_MOTOR);
        // intakeConfig = new TalonFXConfiguration();

        pivotMotorLead = new SparkFlex(PIVOT_MOTOR_LEAD, SparkLowLevel.MotorType.kBrushless);
        pivotMotorFollow = new SparkFlex(PIVOT_MOTOR_FOLLOW, SparkLowLevel.MotorType.kBrushless);
        pivotConfig = new SparkFlexConfig();
        pivotController = pivotMotorLead.getClosedLoopController();

        pivotConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        pivotConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(1, ClosedLoopSlot.kSlot0)
            .i(0, ClosedLoopSlot.kSlot0)
            .d(0, ClosedLoopSlot.kSlot0)
            .feedForward.kCos(1.2, ClosedLoopSlot.kSlot0);
        pivotConfig.closedLoop.maxMotion
            .maxAcceleration(250, ClosedLoopSlot.kSlot0) // REDUCED from 200 — slower ramp
            .allowedProfileError(5, ClosedLoopSlot.kSlot0)
            .cruiseVelocity(250, ClosedLoopSlot.kSlot0) // Changed from 1000 to match test setpoint
            .allowedProfileError(5, ClosedLoopSlot.kSlot0);

        pivotConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(1, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .feedForward.kCos(1, ClosedLoopSlot.kSlot1);
        pivotConfig.closedLoop.maxMotion
            .maxAcceleration(300, ClosedLoopSlot.kSlot1) // REDUCED from 200 — slower ramp
            .allowedProfileError(1, ClosedLoopSlot.kSlot1)
            .cruiseVelocity(300, ClosedLoopSlot.kSlot1) // Changed from 1000 to match test setpoint
            .allowedProfileError(1, ClosedLoopSlot.kSlot1);

        pivotMotorLead.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        SparkFlexConfig followConfig = new SparkFlexConfig();
        // followConfig.inverted(false);
        
        followConfig.follow(PIVOT_MOTOR_LEAD,true);
        pivotMotorFollow.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        intakeVelocityControl = new VelocityTorqueCurrentFOC(0.0);
        intakeVelocityControl.UpdateFreqHz = 0.0;

        // Configure TalonFX intake motor
        configureRollers();

    //     intakeConfig.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;
    //     intakeConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
    //     intakeConfig.CurrentLimits.SupplyCurrentLimit = 40;
    //     intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    //     // Velocity control PID (slot 0)
    //     intakeConfig.Slot0.kP = 0.12;
    //     intakeConfig.Slot0.kI = 0.0;
    //     intakeConfig.Slot0.kD = 0.0;
    //     intakeConfig.Slot0.kS = 0.5;
    //     intakeConfig.Slot0.kV = 0.12; // velocity feedforward

    //     intakeMotor.getConfigurator().apply(intakeConfig);
    //    PhoenixUtil.run(() -> intakeMotor.clearStickyFaults());
    //     PhoenixUtil.run(() -> intakeMotor.getConfigurator().apply(intakeConfig));

        tunables.add("Intake Motor", intakeMotor);
        tunables.add("Pivot Motor Lead", pivotMotorLead);
         tunables.add("Pivot Motor Follower", pivotMotorFollow);

        // System check: watch these devices for faults and register the system check
        FaultReporter.getInstance().registerHardware(SYSTEM_CHECK_NAME, "intake roller", intakeMotor);
        FaultReporter.getInstance().registerHardware(SYSTEM_CHECK_NAME, "pivot lead", pivotMotorLead);
        FaultReporter.getInstance().registerHardware(SYSTEM_CHECK_NAME, "pivot follower", pivotMotorFollow);
        FaultReporter.getInstance().registerSystemCheck(SYSTEM_CHECK_NAME, this::systemCheck);
    }

    @NotLogged
    public Command moveIntake(boolean reverse) {
        return commandBuilder()
            .onExecute(() ->{
 var maxVelocity = new MotionMagicVelocityVoltage(intakeSpeed.get())
               .withAcceleration(intakeAcceleration.getAsInt());

               intakeMotor.setControl(maxVelocity);
            } )
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
        // double speed = reverse ? -this.intakeSpeed.get() : this.intakeSpeed.get();
           intakeVelocityControl.withVelocity( this.intakeSpeed.get());
        // intakeMotor.setControl(intakeVelocityControl);
           if (Math.abs(intakeVelocityControl.Velocity) > 1e-6) {
                    intakeMotor.setControl(intakeVelocityControl);
                } else {
                    intakeMotor.stopMotor();
                }
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
        return sequence( extendIntake().withTimeout(0.6),
                        retrackIntake().withTimeout(0.6))
            .repeatedly()
            .withName("Intake.agitate()");
    }


    @NotLogged
    public Command retrackIntake() {
        return commandBuilder().onExecute(() -> this.moveMotorPosIn(0));
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
    public Command stopPivot() {
        // pivotMotorLead.setVoltage(0);
        return commandBuilder().onExecute(() -> pivotMotorLead.setVoltage(0));
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

      private void configureRollers() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 170.0;
        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = .5;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = 0.0;
        config.Slot0.kS = 10.0;
        config.Slot0.kV = 0.0;
        config.Slot0.kA = 0.0;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        PhoenixUtil.run(() -> intakeMotor.clearStickyFaults());
        PhoenixUtil.run(() -> intakeMotor.getConfigurator().apply(config));

      
    }

    // ------------------------------------------------------------------
    // System check (see lib/systemcheck/README.md)
    // ------------------------------------------------------------------

    /** How close the pivot must get to its target, in motor rotations. */
    private static final double CHECK_PIVOT_TOLERANCE = 0.15;
    /** How long each pivot move gets to reach its target, in seconds. */
    private static final double CHECK_PIVOT_TIMEOUT = 2.0;
    /** Only compare the two pivot motors if the leader drew at least this much current, in amps. */
    private static final double CHECK_PIVOT_MIN_CURRENT = 5.0;
    /** The follower must draw at least this fraction of the leader's peak current. */
    private static final double CHECK_FOLLOWER_CURRENT_RATIO = 0.5;

    /**
     * Intake system check:
     * <ol>
     *   <li>Spins the roller open-loop at 3 V and checks it turns.</li>
     *   <li>Retracts, extends, then retracts the pivot using the normal
     *       {@link #retrackIntake()} / {@link #extendIntake()} commands, and checks
     *       each move reaches its position in time.</li>
     *   <li>During each pivot move, compares the peak current of the two pivot
     *       motors. If the follower isn't drawing current while the leader is,
     *       the follower isn't helping (unplugged motor wires, bad follower
     *       config), even if the leader alone can still move the pivot.</li>
     * </ol>
     * <b>The intake needs room to extend.</b>
     */
    public Command systemCheck() {
        return Commands.sequence(
            SystemChecks.spinTalonFX(SYSTEM_CHECK_NAME, this, "intake roller", intakeMotor, 3.0, 8.0),
            checkPivotMove("retract", retrackIntake(), 0.0),
            checkPivotMove("extend", extendIntake(), pivotPosition.get()),
            checkPivotMove("retract", retrackIntake(), 0.0)
        ).withName("Intake.systemCheck()");
    }

    private Command checkPivotMove(String label, Command moveCommand, double target) {
        double[] peakCurrent = { 0.0, 0.0 }; // leader, follower

        Command reset = Commands.runOnce(() -> {
            peakCurrent[0] = 0.0;
            peakCurrent[1] = 0.0;
        });

        // Runs alongside the move (no requirements) and records the highest current each motor draws.
        Command sampleCurrent = Commands.run(() -> {
            peakCurrent[0] = Math.max(peakCurrent[0], Math.abs(pivotMotorLead.getOutputCurrent()));
            peakCurrent[1] = Math.max(peakCurrent[1], Math.abs(pivotMotorFollow.getOutputCurrent()));
        });

        Command move = SystemChecks.moveToPosition(
            SYSTEM_CHECK_NAME,
            "pivot " + label,
            moveCommand,
            () -> pivotMotorLead.getEncoder().getPosition(),
            target,
            CHECK_PIVOT_TOLERANCE,
            CHECK_PIVOT_TIMEOUT,
            "rot"
        );

        Command compareCurrent = Commands.runOnce(() -> {
            double lead = peakCurrent[0];
            double follow = peakCurrent[1];
            if (lead >= CHECK_PIVOT_MIN_CURRENT && follow < CHECK_FOLLOWER_CURRENT_RATIO * lead) {
                FaultReporter.getInstance()
                    .addFault(
                        SYSTEM_CHECK_NAME,
                        String.format(
                            "[pivot follower] drew only %.1f A while the leader drew %.1f A (%s) - check follower motor wires and follow config",
                            follow,
                            lead,
                            label
                        )
                    );
            }
        });

        return Commands.sequence(reset, Commands.deadline(move, sampleCurrent), compareCurrent);
    }
}
