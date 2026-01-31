package org.team1126.robot.util;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import org.team1126.lib.util.Alliance;
import org.team1126.robot.Robot;

public class MatchData {

    private boolean isHubActive;
    private boolean hubActive1And3;
    private int currentHubShift;

    private BooleanPublisher isHubActivePublisher;
    private IntegerPublisher currentHubShiftPublisher;
    private DoublePublisher matchTimePublisher;

    public MatchData() {
        isHubActive = false;
        hubActive1And3 = false;
        currentHubShift = -2;

        NetworkTableInstance nti = NetworkTableInstance.getDefault();
        NetworkTable table = nti.getTable("FMSInfo");

        isHubActivePublisher = table.getBooleanTopic("isHubActive").publish();
        currentHubShiftPublisher = table.getIntegerTopic("currentHubShift").publish();
        matchTimePublisher = table.getDoubleTopic("matchTime").publish();
    }

    public boolean isHubActive() {
        int hubShift = getCurrentHubShift();
        boolean isActive1And3 = isHubActive1And3();

        if (isActive1And3) {
            if (hubShift == 2 || hubShift == 4) {
                isHubActive = false;
            }
        } else {
            if (hubShift == 1 || hubShift == 3) {
                isHubActive = false;
            }
        }
        isHubActivePublisher.set(isHubActive);
        return isHubActive;
    }

    private int getCurrentHubShift() {
        double currentMatchTime = Robot.matchTime();
        matchTimePublisher.set(currentMatchTime);

        if (DriverStation.isAutonomous()) {
            currentHubShift = -1;
        } else if (currentMatchTime <= 140 && currentMatchTime >= 130) {
            currentHubShift = 0;
        } else if (currentMatchTime < 130 && currentMatchTime >= 105) {
            currentHubShift = 1;
        } else if (currentMatchTime < 105 && currentMatchTime >= 80) {
            currentHubShift = 2;
        } else if (currentMatchTime < 80 && currentMatchTime >= 55) {
            currentHubShift = 3;
        } else if (currentMatchTime < 55 && currentMatchTime >= 30) {
            currentHubShift = 4;
        } else if (currentMatchTime < 30) {
            currentHubShift = 5;
        } else {
            currentHubShift = -2;
        }
        currentHubShiftPublisher.set(currentHubShift);
        return currentHubShift;
    }

    /**
     * Game specific message will either be an empty string (not sent yet), an 'R', or a 'B'.
     * The charcter it sends is the alliance who will be inactive first and will be active in
     * shifts 2 and 4. The other alliance will be active in shifts 1 and 3.
     * @return if our hub is inactive first
     */
    private boolean isHubActive1And3() {
        boolean isBlueAlliance = Alliance.isBlue();
        String gameMessage = DriverStation.getGameSpecificMessage();

        if (gameMessage.length() < 1) {
            return false;
        }
        char gameMessageChar = gameMessage.charAt(0);

        if (isBlueAlliance && gameMessageChar == 'B') {
            hubActive1And3 = false;
        } else if (isBlueAlliance && gameMessageChar == 'R') {
            hubActive1And3 = true;
        } else if (!isBlueAlliance && gameMessageChar == 'B') {
            hubActive1And3 = true;
        } else if (!isBlueAlliance && gameMessageChar == 'R') {
            hubActive1And3 = false;
        } else {
            hubActive1And3 = false;
        }
        return hubActive1And3;
    }
}
