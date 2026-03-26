package org.team1126.robot.subsystems;

import static org.team1126.robot.Constants.FEEDER_MOTOR;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.util.command.GRRSubsystem;

public final class Feeder extends GRRSubsystem {

    private static final TunableTable tunables = Tunables.getNested("feeder");

    private final SparkMax feederMotor;
    private final SparkMaxConfig feederConfig;
    private final RelativeEncoder feederEncoder;
    private final Tunables.TunableInteger feederSpeed = tunables.value("Feeder Speed", 200);
    private final Tunables.TunableInteger feederUnJamSpeed = tunables.value("Feeder UnJam Speed", 85);
    private final SparkClosedLoopController feederController;
    private final Tunables.TunableInteger feederVelocitySetpoint = tunables.value("Feeder Setpoint", 2600);

    public Feeder() {
        feederMotor = new SparkMax(FEEDER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        feederController = feederMotor.getClosedLoopController();
        feederEncoder = feederMotor.getEncoder();
        feederConfig = new SparkMaxConfig();
        feederConfig.closedLoop.p(0).i(0).d(0).feedbackSensor(FeedbackSensor.kPrimaryEncoder).feedForward.kV(.05);

        feederConfig
            .inverted(true)
            .closedLoop.maxMotion.maxAcceleration(875)
            .allowedProfileError(5)
            .cruiseVelocity(175)
            .allowedProfileError(5);

        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        tunables.add("Feeder Motor", feederMotor);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder Velocity", feederEncoder.getVelocity());
        SmartDashboard.putBoolean("Feeder is at setpoint?", isReady());
    }

    /**
     * Finds out if the feeder has reached the target speed
     * @return Whether or not the feeder is ready
     */
    public boolean isReady() {
        if (this.feederEncoder.getVelocity() < this.feederVelocitySetpoint.get() + 100
            && this.feederEncoder.getVelocity() > this.feederVelocitySetpoint.get() - 100) {
            return true;
        }
        return false;
    }

    /**
     * Ready the feeder to feed into the shooter
     */
    public Command readyFeeder() {
        return commandBuilder().onExecute(() ->
            feederController.setSetpoint(this.feederSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl)
        );
    }

    /**
     * Feed the shooter continuously
     */
    public Command feedShooter() {
        return commandBuilder().onExecute(() ->
            feederController.setSetpoint(this.feederSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl)
        );
    }

    /**
     * Stop the feeder
     */
    private void stopFeeder() {
        feederController.setSetpoint(0, SparkBase.ControlType.kMAXMotionVelocityControl);
    }

    /**
     * Unjam the feeder by reversing it
     */
    public Command unJamFeeder() {
        return commandBuilder().onExecute(this::feederUnJam).onEnd(this::stopFeeder);
    }

    /**
     * Internal method to run the feeder in reverse to unjam
     */
    private void feederUnJam() {
        feederController.setSetpoint(-this.feederUnJamSpeed.get(), SparkBase.ControlType.kMAXMotionVelocityControl);
    }

    /**
     * Stop the feeder
     */
    public Command feederStop() {
        return commandBuilder().onExecute(this::stopFeeder);
    }
}
