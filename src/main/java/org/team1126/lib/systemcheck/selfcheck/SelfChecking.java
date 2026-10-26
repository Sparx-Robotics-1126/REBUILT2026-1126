package org.team1126.lib.systemcheck.selfcheck;

import java.util.List;
import org.team1126.lib.systemcheck.SubsystemFault;

/**
 * A piece of hardware that can report its own faults. Implementations wrap
 * a vendor device (TalonFX, SparkMax, CANcoder, ...) and translate its fault
 * flags into {@link SubsystemFault}s.
 *
 * <p>{@link #checkForFaults()} is called by the
 * {@link org.team1126.lib.systemcheck.FaultReporter FaultReporter} about once
 * a second while the robot is disabled, and at the end of every system check.
 * It must be quick and must not block.
 */
public interface SelfChecking {
    /**
     * Returns the faults the device is currently reporting. Return an empty
     * list if everything is fine.
     */
    List<SubsystemFault> checkForFaults();

    /**
     * Clears any sticky (latched) faults stored on the device.
     * Optional; the default does nothing.
     */
    default void clearStickyFaults() {}
}
