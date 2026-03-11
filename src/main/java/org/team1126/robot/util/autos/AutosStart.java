package org.team1126.robot.util.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.team1126.lib.math.geometry.ExtPose;

public enum AutosStart {
    LEFT(2.54, 5.23, 23.85, "Starting LEFT - ", "SL"),
    CENTER(2.287, 4.038, 0.0, "Starting CENTER - ", "SC"),
    RIGHT(3.131, 2.228, 51.13, "Starting RIGHT - ", "SR"),
    OTHER(0.0, 0.0, 0.0, "", "");

    private final ExtPose startingPoint;
    private final String description;
    private final String abbreviation;

    AutosStart(double startingX, double startingY, double startingRot, String description, String abbreviation) {
        this.startingPoint = new ExtPose(startingX, startingY, new Rotation2d(startingRot));
        this.description = description;
        this.abbreviation = abbreviation;
    }

    public Pose2d getStartingPoint() {
        return startingPoint.get();
    }

    public String display(boolean succinct) {
        return succinct ? abbreviation : description;
    }
}
