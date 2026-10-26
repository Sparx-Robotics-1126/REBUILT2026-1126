package org.team1126.lib.systemcheck.selfcheck;

import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagFaults;
import java.util.ArrayList;
import java.util.List;
import org.team1126.lib.systemcheck.SubsystemFault;

/**
 * Self-check for a Redux Robotics Canandmag encoder.
 */
public final class SelfCheckingCanandmag implements SelfChecking {

    private final String label;
    private final Canandmag encoder;

    /**
     * @param label The name shown in fault messages, e.g. "frontLeft encoder".
     * @param encoder The Canandmag.
     */
    public SelfCheckingCanandmag(String label, Canandmag encoder) {
        this.label = label;
        this.encoder = encoder;
    }

    @Override
    public List<SubsystemFault> checkForFaults() {
        List<SubsystemFault> faults = new ArrayList<>();

        if (!encoder.isConnected()) {
            faults.add(SubsystemFault.error("[" + label + "] not responding on CAN - check CAN wiring and power"));
            return faults;
        }

        CanandmagFaults active = encoder.getActiveFaults();
        if (active.faultsValid()) {
            if (active.hardwareFault()) faults.add(SubsystemFault.error("[" + label + "] hardware fault"));
            if (active.magnetOutOfRange()) faults.add(SubsystemFault.error("[" + label + "] magnet out of range - check the magnet"));
            if (active.canIDConflict()) faults.add(SubsystemFault.error("[" + label + "] CAN ID conflict"));
            if (active.canGeneralError()) faults.add(SubsystemFault.error("[" + label + "] CAN error"));
            if (active.underVolt()) faults.add(SubsystemFault.warning("[" + label + "] supply voltage low"));
            if (active.outOfTemperatureRange()) faults.add(SubsystemFault.warning("[" + label + "] out of temperature range"));
        }

        CanandmagFaults sticky = encoder.getStickyFaults();
        if (sticky.faultsValid()) {
            if (sticky.powerCycle()) faults.add(SubsystemFault.warning("[" + label + "] [STICKY] power cycled"));
            if (sticky.magnetOutOfRange()) faults.add(SubsystemFault.warning("[" + label + "] [STICKY] magnet was out of range"));
        }

        return faults;
    }

    @Override
    public void clearStickyFaults() {
        encoder.clearStickyFaults();
    }
}
