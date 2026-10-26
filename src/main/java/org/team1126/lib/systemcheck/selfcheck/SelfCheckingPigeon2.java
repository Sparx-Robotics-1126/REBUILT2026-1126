package org.team1126.lib.systemcheck.selfcheck;

import com.ctre.phoenix6.hardware.Pigeon2;

/**
 * Self-check for a CTRE Pigeon 2.0 gyro.
 */
public final class SelfCheckingPigeon2 extends PhoenixSelfChecking {

    /**
     * @param label The name shown in fault messages, e.g. "gyro".
     * @param pigeon The Pigeon2.
     */
    public SelfCheckingPigeon2(String label, Pigeon2 pigeon) {
        super(label, pigeon, pigeon::clearStickyFaults);

        active(pigeon.getFault_Hardware(), "hardware failure", false);
        active(pigeon.getFault_BootupAccelerometer(), "accelerometer failed boot checks", false);
        active(pigeon.getFault_BootupGyroscope(), "gyroscope failed boot checks", false);
        active(pigeon.getFault_BootDuringEnable(), "rebooted while enabled - heading may have reset", false);

        active(pigeon.getFault_BootIntoMotion(), "robot was moving when the gyro booted - power cycle it", true);
        active(pigeon.getFault_BootupMagnetometer(), "magnetometer failed boot checks", true);
        active(pigeon.getFault_DataAcquiredLate(), "sensor data acquired late", true);
        active(pigeon.getFault_LoopTimeSlow(), "internal loop running slow", true);
        active(pigeon.getFault_SaturatedGyroscope(), "gyroscope saturated (spun too fast)", true);
        active(pigeon.getFault_SaturatedAccelerometer(), "accelerometer saturated (big hit)", true);
        active(pigeon.getFault_Undervoltage(), "supply voltage low", true);
        active(pigeon.getFault_UnlicensedFeatureInUse(), "Phoenix Pro feature used without a license", true);

        sticky(pigeon.getStickyFault_BootDuringEnable(), "rebooted while enabled - heading may have reset");
        sticky(pigeon.getStickyFault_Hardware(), "hardware failure");
        sticky(pigeon.getStickyFault_SaturatedGyroscope(), "gyroscope saturated");
    }
}
