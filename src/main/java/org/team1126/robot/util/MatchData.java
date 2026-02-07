package org.team1126.robot.util;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1126.lib.util.Alliance;

public class MatchData {

    private static BooleanPublisher shouldIShootPublisher;
    private static DoublePublisher matchTimePublisher;
    private static StringPublisher gameSpecificMessagePublisher;
    private static String whoStartedFirst;

    public MatchData() {
        whoStartedFirst = getGameSpecificMessage();

        NetworkTableInstance nti = NetworkTableInstance.getDefault();
        NetworkTable table = nti.getTable("FMSInfo");

        shouldIShootPublisher = table.getBooleanTopic("shouldIShoot").publish();
        matchTimePublisher = table.getDoubleTopic("matchTime").publish();
        gameSpecificMessagePublisher = table.getStringTopic("GameSpecificMessage").publish();
    }

    public static boolean shouldIShoot() {
        double time = DriverStation.getMatchTime();
        // String whoStartedFirst = getGameSpecificMessage();
        if (!DriverStation.isFMSAttached()) {
            gameSpecificMessagePublisher.set(whoStartedFirst);
        }
        boolean blue = Alliance.isBlue();
        boolean winAuto = (blue && "B".equals(whoStartedFirst)) || (!blue && "R".equals(whoStartedFirst));
        SmartDashboard.putBoolean("winAuto", winAuto);
        matchTimePublisher.set(time);

        if (time < 140 && time > 130) {
            // auto and transition, everyone is active
            shouldIShootPublisher.set(true);
            return true;
        } else if (time < 130 && time > 105) {
            // shift 1
            shouldIShootPublisher.set(!winAuto);
            return !winAuto;
        } else if (time < 105 && time > 80) {
            // shift 2
            shouldIShootPublisher.set(winAuto);
            return winAuto;
        } else if (time < 80 && time > 55) {
            // shift 3
            shouldIShootPublisher.set(!winAuto);
            return !winAuto;
        } else if (time < 55 && time > 30) {
            // shift 4
            shouldIShootPublisher.set(winAuto);
            return winAuto;
        } else if (time < 30 && time > 0) {
            // end game, everyone is active
            shouldIShootPublisher.set(true);
            return true;
        }
        shouldIShootPublisher.set(false);
        if (!DriverStation.isFMSAttached() && DriverStation.isDisabled()) {
            whoStartedFirst = getGameSpecificMessage();
        }
        return false;
    }

    private static String getGameSpecificMessage() {
        String message = DriverStation.getGameSpecificMessage();
        if (!DriverStation.isFMSAttached() && message.length() == 0) {
            int randomInt = (int) Math.floor(Math.random() * 2);
            if (randomInt == 0) {
                return "R";
            } else {
                return "B";
            }
        }
        return message;
    }
}
