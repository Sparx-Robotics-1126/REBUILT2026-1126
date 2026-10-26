package org.team1126.lib.systemcheck.selfcheck;

import edu.wpi.first.wpilibj.RobotController;
import java.util.List;
import org.team1126.lib.systemcheck.SubsystemFault;

/**
 * Checks the battery voltage. Most useful while the robot is disabled, when
 * the reading is close to the battery's resting voltage.
 */
public final class SelfCheckingBattery implements SelfChecking {

    private final double warningVolts;
    private final double errorVolts;

    /**
     * @param warningVolts Below this, warn that the battery should be swapped (e.g. 12.3).
     * @param errorVolts Below this, report an error (e.g. 11.8).
     */
    public SelfCheckingBattery(double warningVolts, double errorVolts) {
        this.warningVolts = warningVolts;
        this.errorVolts = errorVolts;
    }

    @Override
    public List<SubsystemFault> checkForFaults() {
        // Fixed text (no live voltage) so the same fault isn't re-created every poll.
        double volts = RobotController.getBatteryVoltage();
        if (volts < errorVolts) {
            return List.of(SubsystemFault.error(String.format("[battery] below %.1f V - swap the battery", errorVolts)));
        }
        if (volts < warningVolts) {
            return List.of(SubsystemFault.warning(String.format("[battery] below %.1f V - swap the battery", warningVolts)));
        }
        return List.of();
    }
}
