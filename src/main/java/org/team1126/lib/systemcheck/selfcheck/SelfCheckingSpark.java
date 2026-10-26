package org.team1126.lib.systemcheck.selfcheck;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import java.util.ArrayList;
import java.util.List;
import org.team1126.lib.systemcheck.SubsystemFault;

/**
 * Self-check for a REV SPARK MAX or SPARK Flex.
 */
public final class SelfCheckingSpark implements SelfChecking {

    private final String label;
    private final SparkBase spark;
    private final Debouncer disconnectedDebounce = new Debouncer(1.5, DebounceType.kRising);

    /**
     * @param label The name shown in fault messages, e.g. "storage roller".
     * @param spark The SparkMax or SparkFlex.
     */
    public SelfCheckingSpark(String label, SparkBase spark) {
        this.label = label;
        this.spark = spark;
    }

    @Override
    public List<SubsystemFault> checkForFaults() {
        List<SubsystemFault> faults = new ArrayList<>();

        // REV has no "is connected" call. Read a status value and check the
        // error it set; anything other than kOk means we aren't hearing from it.
        spark.getBusVoltage();
        REVLibError error = spark.getLastError();
        boolean noResponse = error != REVLibError.kOk;
        if (disconnectedDebounce.calculate(noResponse)) {
            faults.add(
                SubsystemFault.error(
                    String.format(
                        "[%s] not responding on CAN (ID %d, %s) - check CAN wiring, power and breaker",
                        label,
                        spark.getDeviceId(),
                        error.name()
                    )
                )
            );
            return faults;
        }

        // Active faults (errors unless noted)
        Faults active = spark.getFaults();
        error(faults, active.can, "CAN fault - check CAN wiring");
        error(faults, active.gateDriver, "gate driver fault - controller may be damaged");
        error(faults, active.motorType, "motor type mismatch - check brushed/brushless config");
        error(faults, active.sensor, "sensor fault - check encoder cable");
        error(faults, active.escEeprom, "ESC EEPROM fault");
        error(faults, active.firmware, "firmware fault - update firmware in REV Hardware Client");
        error(faults, active.other, "other fault - check REV Hardware Client");
        warning(faults, active.temperature, "motor overheating - let it cool");

        // Active warnings
        Warnings warnings = spark.getWarnings();
        warning(faults, warnings.brownout, "brownout - supply voltage low");
        warning(faults, warnings.overcurrent, "overcurrent");
        warning(faults, warnings.stall, "motor stalled");
        warning(faults, warnings.sensor, "sensor warning - check encoder cable");
        warning(faults, warnings.extEeprom, "external EEPROM warning");

        // Sticky: happened since faults were last cleared (e.g. during the last match).
        Faults sticky = spark.getStickyFaults();
        warning(faults, sticky.can, "[STICKY] CAN fault - check CAN wiring");
        warning(faults, sticky.gateDriver, "[STICKY] gate driver fault");
        warning(faults, sticky.sensor, "[STICKY] sensor fault - check encoder cable");
        warning(faults, sticky.temperature, "[STICKY] motor overheated");
        Warnings stickyWarnings = spark.getStickyWarnings();
        warning(faults, stickyWarnings.hasReset, "[STICKY] controller reset - check power wiring and breaker");
        warning(faults, stickyWarnings.brownout, "[STICKY] brownout");

        return faults;
    }

    @Override
    public void clearStickyFaults() {
        spark.clearFaults();
    }

    private void error(List<SubsystemFault> faults, boolean condition, String description) {
        if (condition) faults.add(SubsystemFault.error("[" + label + "] " + description));
    }

    private void warning(List<SubsystemFault> faults, boolean condition, String description) {
        if (condition) faults.add(SubsystemFault.warning("[" + label + "] " + description));
    }
}
