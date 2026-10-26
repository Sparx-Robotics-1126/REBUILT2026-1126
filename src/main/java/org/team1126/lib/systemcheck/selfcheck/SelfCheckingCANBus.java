package org.team1126.lib.systemcheck.selfcheck;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import java.util.ArrayList;
import java.util.List;
import org.team1126.lib.systemcheck.SubsystemFault;

/**
 * Self-check for a whole CAN bus (the roboRIO bus or a CANivore). Catches an
 * unplugged CANivore, an overloaded bus, and wiring problems that cause
 * bus-off events.
 */
public final class SelfCheckingCANBus implements SelfChecking {

    private static final double HIGH_UTILIZATION = 0.90;

    private final String label;
    private final CANBus bus;
    private int lastBusOffCount = -1;
    private boolean busOffSeen = false;

    /**
     * @param label The name shown in fault messages, e.g. "LowerCAN".
     * @param bus The CAN bus.
     */
    public SelfCheckingCANBus(String label, CANBus bus) {
        this.label = label;
        this.bus = bus;
    }

    @Override
    public List<SubsystemFault> checkForFaults() {
        List<SubsystemFault> faults = new ArrayList<>();
        CANBusStatus status = bus.getStatus();

        if (!status.Status.isOK()) {
            faults.add(
                SubsystemFault.error(
                    "[" + label + "] CAN bus not available (" + status.Status.getName() + ") - is the CANivore plugged in?"
                )
            );
            return faults;
        }

        if (status.BusUtilization > HIGH_UTILIZATION) {
            faults.add(
                SubsystemFault.warning(
                    String.format("[%s] CAN bus utilization high (%.0f%%)", label, status.BusUtilization * 100.0)
                )
            );
        }

        if (lastBusOffCount >= 0 && status.BusOffCount > lastBusOffCount) {
            busOffSeen = true;
        }
        lastBusOffCount = status.BusOffCount;
        // Stays reported until ClearAllFaults, so a short burst isn't missed.
        if (busOffSeen) {
            faults.add(
                SubsystemFault.warning(
                    "[" + label + "] CAN bus-off events detected - check wiring and termination"
                )
            );
        }

        return faults;
    }

    @Override
    public void clearStickyFaults() {
        busOffSeen = false;
    }
}
