package org.team1126.robot.util;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.tunable.Tunables.TunableBoolean;
import org.team1126.lib.util.Alliance;
import org.team1126.robot.Robot;

/**
 * Tracks hub shift data.
 */
@Logged
public final class ShiftTracker {

    private static final TunableBoolean shiftDefaultWin = Tunables.value("shiftDefaultWin", true);


    public ShiftTracker() {}

    /**
     * Returns {@code true} if our alliance's hub is active.
     */
    public boolean active() {
        if (DriverStation.isDisabled()) return false;
        if (DriverStation.isAutonomous()) return true;

        double timeLeft = Robot.matchTime(); // counts down from 140 to 0 in teleop
        double time = 140.0 - timeLeft; // convert to count-up for shift logic
        if (time < 10.0 || time >= 110.0) return true;

        boolean wonAuto = wonAuto();
        if (time >= 85.0) return wonAuto;
        if (time >= 60.0) return !wonAuto;
        if (time >= 35.0) return wonAuto;
        if (time >= 10.0) return !wonAuto;

        return false;
    }

    // Only used for the method below
    private static final double[] SHIFTS = { 0.0, 10.0, 35.0, 60.0, 85.0, 110.0, 140.0 };

    /**
     * Returns the number of seconds left in the current shift period.
     */
    public double shiftTimeLeft() {
        if (DriverStation.isDisabled()) return 0.0;
        if (DriverStation.isAutonomous()) return Robot.matchTime(); // counts down from 20 to 0

        double timeLeft = Robot.matchTime(); // counts down from 140 to 0 in teleop
        double time = 140.0 - timeLeft; // convert to count-up for shift logic

        for (int i = SHIFTS.length - 2; i >= 0; i--) {
            if (time >= SHIFTS[i]) return SHIFTS[i + 1] - time;
        }

        return 0.0;
    }

    /**
     * The amount of time left in the current period of the match (auto, teleop).
     */
    public double matchTimeLeft() {
        if (DriverStation.isDisabled()) return 0.0;
        return Math.max(Robot.matchTime(), 0.0); // counts down in both modes
    }

    /**
     * Returns {@code true} if our alliance won auto.
     */
    public boolean wonAuto() {
        boolean wonAuto = shiftDefaultWin.get();
        String message = DriverStation.getGameSpecificMessage();
        if (!message.isEmpty()) {
            char c = message.charAt(0);
            if (c == 'B') wonAuto = Alliance.isBlue();
            else if (c == 'R') wonAuto = Alliance.isRed();
        }

        return wonAuto;
    }
}
