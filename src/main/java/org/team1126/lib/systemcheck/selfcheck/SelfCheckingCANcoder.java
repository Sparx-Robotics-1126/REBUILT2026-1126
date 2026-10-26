package org.team1126.lib.systemcheck.selfcheck;

import com.ctre.phoenix6.hardware.CANcoder;

/**
 * Self-check for a CTRE CANcoder (e.g. swerve module absolute encoders).
 */
public final class SelfCheckingCANcoder extends PhoenixSelfChecking {

    /**
     * @param label The name shown in fault messages, e.g. "frontLeft encoder".
     * @param encoder The CANcoder.
     */
    public SelfCheckingCANcoder(String label, CANcoder encoder) {
        super(label, encoder, encoder::clearStickyFaults);

        active(encoder.getFault_Hardware(), "hardware failure", false);
        active(encoder.getFault_BadMagnet(), "magnet too far or too close - check the magnet", false);
        active(encoder.getFault_BootDuringEnable(), "rebooted while enabled - check power and CAN wiring", false);
        active(encoder.getFault_Undervoltage(), "supply voltage low", true);
        active(encoder.getFault_UnlicensedFeatureInUse(), "Phoenix Pro feature used without a license", true);

        sticky(encoder.getStickyFault_BootDuringEnable(), "rebooted while enabled - check power and CAN wiring");
        sticky(encoder.getStickyFault_BadMagnet(), "magnet was out of range");
        sticky(encoder.getStickyFault_Hardware(), "hardware failure");
    }
}
