package org.team1126.robot.subsystems;

import static org.team1126.robot.util.ShootParams.hoodPositionMap;

import java.util.function.DoubleSupplier;

import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.tunable.Tunables.TunableDouble;
import org.team1126.lib.util.command.GRRSubsystem;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;

public final class Hood extends GRRSubsystem {
    private static final TunableTable tunables = Tunables.getNested("hood");
    private static final TunableDouble stallVelocity = tunables.value("stallVelocity", 0.05);
    private static final TunableDouble homingVelocity = tunables.value("homingVelocity", -30.0);
    private static final TunableDouble zeroZero = tunables.value("zeroZero", 1.0); // In rotations per second

    private final TalonFX motor;

    private final VelocityTorqueCurrentFOC motorVelocityControl;
    private final TunableDouble motorVelocity;
    private final TunableDouble motorPosition;
    

    private boolean isZeroed = false;

    public Hood() {
        this.motor = new TalonFX(0);

        this.motorVelocity = tunables.value("velocity", 0.254);
        this.motorPosition = tunables.value("position", .25);

        motorVelocityControl = new VelocityTorqueCurrentFOC(0.0);
        motorVelocityControl.UpdateFreqHz = 0.0;
    }

    /**
     * Run the hood pivot to target a specific distance based on a preset interpolating map.
     * @param distance The distance to target in meters.
     */
    public Command targetDistance(final DoubleSupplier distance) {
        return goTo(() -> hoodPositionMap.get(distance.getAsDouble())).withName("Hood.targetDistance()");
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
                    motorVelocityControl.withVelocity(homingVelocity.get());
                    motor.setControl(motorVelocityControl);
                    if (!debouncer.calculate(Math.abs(motorVelocity.getAsDouble()) < stallVelocity.get())) return;

                    isZeroed = true;
                    motor.setPosition(0.0);
                }

                var maxVelocity = new MotionMagicVoltage(motorPosition.get());
                motor.setControl(maxVelocity);
            })
            .onEnd(motor::stopMotor);
    }
}
