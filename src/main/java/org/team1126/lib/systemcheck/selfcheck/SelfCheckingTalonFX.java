package org.team1126.lib.systemcheck.selfcheck;

import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Self-check for a CTRE TalonFX (Kraken X60 / X44, Falcon 500).
 */
public final class SelfCheckingTalonFX extends PhoenixSelfChecking {

    /**
     * @param label The name shown in fault messages, e.g. "shooter lead".
     * @param motor The TalonFX.
     */
    public SelfCheckingTalonFX(String label, TalonFX motor) {
        super(label, motor, motor::clearStickyFaults);

        // Errors: something is broken right now.
        active(motor.getFault_Hardware(), "hardware failure", false);
        active(motor.getFault_BootDuringEnable(), "rebooted while enabled - check power wiring and breaker", false);
        active(motor.getFault_BridgeBrownout(), "output shut off by low voltage (brownout)", false);
        active(motor.getFault_OverSupplyV(), "supply voltage too high", false);
        active(motor.getFault_UnstableSupplyV(), "supply voltage unstable - check for a loose power connection", false);
        active(motor.getFault_RemoteSensorDataInvalid(), "remote sensor data invalid - check the CANcoder", false);
        active(motor.getFault_FusedSensorOutOfSync(), "fused CANcoder out of sync with rotor", false);
        active(motor.getFault_MissingDifferentialFX(), "differential partner TalonFX missing", false);
        active(motor.getFault_MissingHardLimitRemote(), "remote limit switch device missing", false);
        active(motor.getFault_MissingSoftLimitRemote(), "remote soft limit device missing", false);
        active(motor.getFault_RotorFault1(), "rotor fault 1 - check the motor in Phoenix Tuner", false);
        active(motor.getFault_RotorFault2(), "rotor fault 2 - check the motor in Phoenix Tuner", false);

        // Warnings: still runs, but look at it.
        active(motor.getFault_DeviceTemp(), "motor overheating - let it cool", true);
        active(motor.getFault_ProcTemp(), "controller processor overheating", true);
        active(motor.getFault_Undervoltage(), "supply voltage low - check battery", true);
        active(motor.getFault_RemoteSensorReset(), "remote sensor reset", true);
        active(motor.getFault_StaticBrakeDisabled(), "static brake disabled", true);
        active(motor.getFault_UnlicensedFeatureInUse(), "Phoenix Pro feature used without a license", true);
        active(
            motor.getFault_UsingFusedCANcoderWhileUnlicensed(),
            "fused CANcoder used without a Pro license (falling back to remote)",
            true
        );

        // Sticky: happened since faults were last cleared (e.g. during the last match).
        sticky(motor.getStickyFault_BootDuringEnable(), "rebooted while enabled - check power wiring and breaker");
        sticky(motor.getStickyFault_BridgeBrownout(), "output shut off by low voltage (brownout)");
        sticky(motor.getStickyFault_Hardware(), "hardware failure");
        sticky(motor.getStickyFault_DeviceTemp(), "motor overheated");
        sticky(motor.getStickyFault_ProcTemp(), "controller processor overheated");
        sticky(motor.getStickyFault_Undervoltage(), "supply voltage dropped low");
        sticky(motor.getStickyFault_UnstableSupplyV(), "supply voltage was unstable - check for a loose power connection");
    }
}
