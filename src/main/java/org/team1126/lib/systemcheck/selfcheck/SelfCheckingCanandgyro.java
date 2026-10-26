package org.team1126.lib.systemcheck.selfcheck;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroFaults;
import java.util.ArrayList;
import java.util.List;
import org.team1126.lib.systemcheck.SubsystemFault;

/**
 * Self-check for a Redux Robotics Canandgyro.
 */
public final class SelfCheckingCanandgyro implements SelfChecking {

    private final String label;
    private final Canandgyro gyro;

    /**
     * @param label The name shown in fault messages, e.g. "gyro".
     * @param gyro The Canandgyro.
     */
    public SelfCheckingCanandgyro(String label, Canandgyro gyro) {
        this.label = label;
        this.gyro = gyro;
    }

    @Override
    public List<SubsystemFault> checkForFaults() {
        List<SubsystemFault> faults = new ArrayList<>();

        if (!gyro.isConnected()) {
            faults.add(SubsystemFault.error("[" + label + "] not responding on CAN - check CAN wiring and power"));
            return faults;
        }

        CanandgyroFaults active = gyro.getActiveFaults();
        if (active.faultsValid()) {
            if (active.hardwareFault()) faults.add(SubsystemFault.error("[" + label + "] hardware fault"));
            if (active.canIDConflict()) faults.add(SubsystemFault.error("[" + label + "] CAN ID conflict"));
            if (active.canGeneralError()) faults.add(SubsystemFault.error("[" + label + "] CAN error"));
            if (active.outOfTemperatureRange()) faults.add(SubsystemFault.warning("[" + label + "] out of temperature range"));
            if (active.calibrating()) faults.add(SubsystemFault.warning("[" + label + "] calibrating - keep the robot still"));
            if (active.angularVelocitySaturation()) faults.add(SubsystemFault.warning("[" + label + "] gyro saturated"));
            if (active.accelerationSaturation()) faults.add(SubsystemFault.warning("[" + label + "] accelerometer saturated"));
        }

        CanandgyroFaults sticky = gyro.getStickyFaults();
        if (sticky.faultsValid()) {
            if (sticky.powerCycle()) faults.add(SubsystemFault.warning("[" + label + "] [STICKY] power cycled - heading may have reset"));
            if (sticky.hardwareFault()) faults.add(SubsystemFault.warning("[" + label + "] [STICKY] hardware fault"));
        }

        return faults;
    }

    @Override
    public void clearStickyFaults() {
        gyro.clearStickyFaults();
    }
}
