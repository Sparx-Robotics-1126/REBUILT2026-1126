package org.team1126.robot.subsystems;

import static org.team1126.robot.Constants.FEEDER_MOTOR;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.tunable.Tunables.TunableDouble;
import org.team1126.lib.util.command.GRRSubsystem;
import org.team1126.lib.util.vendors.PhoenixUtil;

public final class Feeder extends GRRSubsystem {

    private static final TunableTable tunables = Tunables.getNested("feeder");

    private final TalonFX feederMotor;
    private TalonFXConfiguration feederConfig;
    private final StatusSignal<Double> closedLoopError;
      private static final TunableDouble atVelocityEpsilon = tunables.value("atVelocityEpsilon", 106.0);
  
    private final Tunables.TunableInteger feederSpeed = tunables.value("Feeder Speed", 200);
    private final Tunables.TunableInteger feederUnJamSpeed = tunables.value("Feeder UnJam Speed", 85);
    private final Tunables.TunableInteger feederVelocitySetpoint = tunables.value("Feeder Setpoint", 2600);

    public Feeder() {
        feederMotor = new TalonFX(FEEDER_MOTOR);
        feederConfig = new TalonFXConfiguration();
        
        feederConfig.Slot0.kP = 0.1;
        feederConfig.Slot0.kI = 0.00001;
        feederConfig.Slot0.kD = 0;
        feederConfig.Slot0.kV = 0.0008;
       
        feederConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        feederConfig.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;

        // feederMotor.getConfigurator().apply(feederConfig);
          


     PhoenixUtil.run(() -> feederMotor.clearStickyFaults());
        PhoenixUtil.run(() -> feederMotor.getConfigurator().apply(feederConfig));

closedLoopError = feederMotor.getClosedLoopError();
        tunables.add("Feeder Motor", feederMotor);
    }

    @Override
    public void periodic() {
          BaseStatusSignal.refreshAll(closedLoopError);
        SmartDashboard.putNumber("Feeder Velocity", feederMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("Feeder is at setpoint?", isReady());
        SmartDashboard.putNumber("Feeder Closed Loop Error", closedLoopError.getValueAsDouble());
    }

    /**
     * Finds out if the feeder has reached the target speed
     * @return Whether or not the feeder is ready
     */
    public boolean isReady() {

         return (Math.abs(closedLoopError.getValueAsDouble()) <= atVelocityEpsilon.get());
        // if (this.feederMotor.getVelocity().getValueAsDouble() < this.feederVelocitySetpoint.get() + 100
        //     && this.feederMotor.getVelocity().getValueAsDouble() > this.feederVelocitySetpoint.get() - 100) {
        //     return true;
        // }
        // return false;
    }

    /**
     * Ready the feeder to feed into the shooter
     */
    public Command readyFeeder() {
        return commandBuilder().onExecute(() ->{
            feederMotor.setControl(new VelocityVoltage(this.feederSpeed.get()));
        }).onEnd(() -> feederMotor.stopMotor());
    }

    /**
     * Feed the shooter continuously
     */
    public Command feedShooter(BooleanSupplier isReady) {

          return commandBuilder()
            .onExecute(() ->  feederMotor.setControl(new VelocityVoltage(this.feederSpeed.get())))
            .onEnd(() -> feederMotor.stopMotor());
        // return commandBuilder().onExecute(() ->{
        //     feederMotor.setControl(new VelocityVoltage(this.feederSpeed.get()));
        // }).onEnd(() -> feederMotor.stopMotor());
    }

    /**
     * Stop the feeder
     */
    private void stopFeeder() {
        feederMotor.setControl(new VelocityVoltage(0));
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
        feederMotor.setControl(new VelocityVoltage(-this.feederUnJamSpeed.get()));
    }

    /**
     * Stop the feeder
     */
    public Command feederStop() {
        return commandBuilder().onExecute(this::stopFeeder);
    }
}
