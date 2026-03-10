package org.team1126.robot.util.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import org.team1126.lib.math.geometry.ExtPose;

public enum AutosStart {
    LEFT(2.54, 5.23, 23.85, "Starting LEFT - "),
    RIGHT(2.287, 4.038, 0.0, "Starting RIGHT - "),
    CENTER(3.131, 2.228, 51.13, "Starting CENTER - "),
    OTHER(0.0, 0.0, 0.0, "");

    private final ExtPose startingPoint;
    private final String description;

    AutosStart(double startingX, double startingY, double startingRot, String description) {
        this.startingPoint = new ExtPose(startingX, startingY, new Rotation2d(startingRot));
        this.description = description;
    }

    public ExtPose getStartingPoint() {
        return startingPoint;
    }

    public String display() {
        return description;
    }
}
