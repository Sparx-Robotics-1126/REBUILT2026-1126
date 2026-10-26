package org.team1126.lib.systemcheck.selfcheck;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import java.util.ArrayList;
import java.util.List;
import org.team1126.lib.systemcheck.SubsystemFault;

/**
 * Shared logic for CTRE Phoenix 6 devices. Subclasses list the fault signals
 * they care about with {@link #active} and {@link #sticky}; this class
 * refreshes them all at once and turns any that are {@code true} into faults.
 *
 * <p>Reading fault signals only reads values the device is already broadcasting,
 * so it does not add CAN traffic.
 */
public abstract class PhoenixSelfChecking implements SelfChecking {

    private record FaultSignal(StatusSignal<Boolean> signal, String description, boolean isWarning) {}

    /** Update rate used if a fault signal was turned off by optimizeBusUtilization(0). */
    private static final double FAULT_SIGNAL_HZ = 4.0;

    protected final String label;
    private final ParentDevice device;
    private final Runnable clearSticky;
    private final List<FaultSignal> faultSignals = new ArrayList<>();
    private final Debouncer disconnectedDebounce = new Debouncer(1.5, DebounceType.kRising);
    private BaseStatusSignal[] signalArray = new BaseStatusSignal[0];

    /**
     * @param label The name shown in fault messages, e.g. "frontLeft move".
     * @param device The Phoenix 6 device.
     * @param clearSticky Clears the device's sticky faults (e.g. {@code talonFX::clearStickyFaults}).
     */
    protected PhoenixSelfChecking(String label, ParentDevice device, Runnable clearSticky) {
        this.label = label;
        this.device = device;
        this.clearSticky = clearSticky;
    }

    /**
     * Adds an active fault signal. Reported while the device says the fault is happening.
     * @param signal The fault signal, e.g. {@code talonFX.getFault_Hardware()}.
     * @param description What the fault means, in plain words.
     * @param isWarning If the fault is only a warning.
     */
    protected final void active(StatusSignal<Boolean> signal, String description, boolean isWarning) {
        add(new FaultSignal(signal, description, isWarning));
    }

    /**
     * Adds a sticky fault signal. Sticky faults stay set after the problem goes
     * away (e.g. the device rebooted during the last match) until they are
     * cleared, so they are reported as warnings.
     * @param signal The sticky fault signal, e.g. {@code talonFX.getStickyFault_BootDuringEnable()}.
     * @param description What the fault means, in plain words.
     */
    protected final void sticky(StatusSignal<Boolean> signal, String description) {
        add(new FaultSignal(signal, "[STICKY] " + description, true));
    }

    private void add(FaultSignal faultSignal) {
        // If someone called optimizeBusUtilization(0) the fault signals stop
        // updating and every check would silently pass. Turn them back on.
        if (faultSignal.signal().getAppliedUpdateFrequency() <= 0.0) {
            faultSignal.signal().setUpdateFrequency(FAULT_SIGNAL_HZ);
        }
        faultSignals.add(faultSignal);
        signalArray = faultSignals.stream().map(FaultSignal::signal).toArray(BaseStatusSignal[]::new);
    }

    @Override
    public List<SubsystemFault> checkForFaults() {
        List<SubsystemFault> faults = new ArrayList<>();

        if (disconnectedDebounce.calculate(!device.isConnected())) {
            faults.add(
                SubsystemFault.error(
                    String.format(
                        "[%s] not responding on CAN (ID %d) - check CAN wiring, power and breaker",
                        label,
                        device.getDeviceID()
                    )
                )
            );
            return faults;
        }

        BaseStatusSignal.refreshAll(signalArray);
        for (FaultSignal faultSignal : faultSignals) {
            if (Boolean.TRUE.equals(faultSignal.signal().getValue())) {
                faults.add(
                    new SubsystemFault(
                        String.format("[%s] %s", label, faultSignal.description()),
                        faultSignal.isWarning()
                    )
                );
            }
        }
        return faults;
    }

    @Override
    public void clearStickyFaults() {
        clearSticky.run();
    }
}
