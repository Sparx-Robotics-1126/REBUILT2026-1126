package org.team1126.lib.systemcheck;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Ready-made building blocks for subsystem system checks.
 *
 * <p>The spin checks run a motor open-loop at a small voltage and confirm it
 * actually turns. Open-loop is used on purpose: it tests the wiring, motor and
 * mechanism without depending on PID tuning, so a check only fails when
 * something is physically wrong (unplugged, tripped breaker, jammed, follower
 * fighting the leader, ...). It can't tell if a motor's direction is
 * configured backwards, because the motor's inversion setting flips the
 * command and the measured speed together - watch the mechanism for that.
 */
public final class SystemChecks {

    /** How long to wait for a motor to spin up before measuring. */
    public static final double DEFAULT_SPIN_UP_SECONDS = 1.0;

    /** A follower must reach at least this fraction of the leader's speed. */
    public static final double FOLLOWER_MIN_RATIO = 0.8;

    private SystemChecks() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Spins a TalonFX (and any followers) open-loop and checks that it turns.
     * The followers must already be following the leader (e.g. via a
     * {@code Follower} control request) and are only measured, not commanded.
     *
     * @param subsystemName The subsystem name used with the {@link FaultReporter}.
     * @param requirement The subsystem that owns the motor, so nothing else drives it during the check.
     * @param label The name shown in fault messages, e.g. "shooter lead".
     * @param motor The motor to spin.
     * @param volts The voltage to apply. Keep it small (2-4 V); the sign sets the direction.
     * @param minRotorRps The slowest acceptable rotor speed, in rotations per second.
     * @param followers Follower motors to compare against the leader.
     */
    public static Command spinTalonFX(
        String subsystemName,
        Subsystem requirement,
        String label,
        TalonFX motor,
        double volts,
        double minRotorRps,
        TalonFX... followers
    ) {
        VoltageOut request = new VoltageOut(0.0);
        DoubleSupplier[] followerSpeeds = new DoubleSupplier[followers.length];
        String[] followerLabels = new String[followers.length];
        for (int i = 0; i < followers.length; i++) {
            TalonFX follower = followers[i];
            followerSpeeds[i] = () -> follower.getVelocity().refresh().getValueAsDouble();
            followerLabels[i] = label + " follower (ID " + follower.getDeviceID() + ")";
        }

        return spin(
            subsystemName,
            requirement,
            label,
            v -> motor.setControl(request.withOutput(v)),
            () -> motor.getVelocity().refresh().getValueAsDouble(),
            motor::stopMotor,
            volts,
            minRotorRps,
            "rot/s",
            followerLabels,
            followerSpeeds
        );
    }

    /**
     * Spins a SparkMax / SparkFlex open-loop and checks that it turns.
     *
     * @param subsystemName The subsystem name used with the {@link FaultReporter}.
     * @param requirement The subsystem that owns the motor.
     * @param label The name shown in fault messages, e.g. "storage roller".
     * @param motor The motor to spin.
     * @param volts The voltage to apply. Keep it small (2-4 V); the sign sets the direction.
     * @param minVelocity The slowest acceptable speed, in the encoder's velocity units
     *                    (RPM unless a velocity conversion factor is configured).
     */
    public static Command spinSpark(
        String subsystemName,
        Subsystem requirement,
        String label,
        SparkBase motor,
        double volts,
        double minVelocity
    ) {
        return spin(
            subsystemName,
            requirement,
            label,
            motor::setVoltage,
            () -> motor.getEncoder().getVelocity(),
            motor::stopMotor,
            volts,
            minVelocity,
            "(encoder units)",
            new String[0],
            new DoubleSupplier[0]
        );
    }

    /**
     * Generic spin check: applies a voltage, waits for the motor to spin up,
     * then checks the measured speed. Use this for motors the helpers above
     * don't cover.
     *
     * @param subsystemName The subsystem name used with the {@link FaultReporter}.
     * @param requirement The subsystem that owns the motor.
     * @param label The name shown in fault messages.
     * @param setVoltage Applies a voltage to the motor.
     * @param velocity Returns the measured velocity.
     * @param stop Stops the motor.
     * @param volts The voltage to apply.
     * @param minVelocity The slowest acceptable speed.
     * @param units Velocity units, for messages.
     * @param followerLabels Names of followers to check (may be empty).
     * @param followerVelocities Measured velocities of the followers (same order as the labels).
     */
    public static Command spin(
        String subsystemName,
        Subsystem requirement,
        String label,
        DoubleConsumer setVoltage,
        DoubleSupplier velocity,
        Runnable stop,
        double volts,
        double minVelocity,
        String units,
        String[] followerLabels,
        DoubleSupplier[] followerVelocities
    ) {
        Runnable measure = () -> {
            FaultReporter reporter = FaultReporter.getInstance();
            double speed = velocity.getAsDouble();

            if (Math.abs(speed) < minVelocity) {
                reporter.addFault(
                    subsystemName,
                    String.format(
                        "[%s] did not spin: %.1f %s at %.1f V (expected at least %.1f) - check power, CAN, breaker or a jam",
                        label,
                        speed,
                        units,
                        volts,
                        minVelocity
                    )
                );
                return;
            }
            for (int i = 0; i < followerVelocities.length; i++) {
                double followerSpeed = followerVelocities[i].getAsDouble();
                if (Math.abs(followerSpeed) < FOLLOWER_MIN_RATIO * Math.abs(speed)) {
                    reporter.addFault(
                        subsystemName,
                        String.format(
                            "[%s] only at %.1f %s while leader is at %.1f - check follower wiring/config or that it isn't fighting the leader",
                            followerLabels[i],
                            followerSpeed,
                            units,
                            speed
                        )
                    );
                }
            }
        };

        return Commands.deadline(
            Commands.sequence(Commands.waitSeconds(DEFAULT_SPIN_UP_SECONDS), Commands.runOnce(measure)),
            Commands.run(() -> setVoltage.accept(volts), requirement)
        )
            .finallyDo(stop::run)
            .withName(subsystemName + ".spinCheck(" + label + ")");
    }

    /**
     * Creates a command that records a fault if a measured value isn't within
     * a tolerance of its target. Useful inside custom system checks.
     *
     * @param subsystemName The subsystem name used with the {@link FaultReporter}.
     * @param description What is being measured, e.g. "[hood] position".
     * @param measured Returns the measured value.
     * @param target The expected value.
     * @param tolerance The allowed difference.
     * @param units Units for the message.
     */
    public static Command expectNear(
        String subsystemName,
        String description,
        DoubleSupplier measured,
        double target,
        double tolerance,
        String units
    ) {
        return Commands.runOnce(() -> {
            double value = measured.getAsDouble();
            if (Math.abs(value - target) > tolerance) {
                FaultReporter.getInstance()
                    .addFault(
                        subsystemName,
                        String.format(
                            "%s is %.2f %s, expected %.2f +/- %.2f",
                            description,
                            value,
                            units,
                            target,
                            tolerance
                        )
                    );
            }
        });
    }
}
