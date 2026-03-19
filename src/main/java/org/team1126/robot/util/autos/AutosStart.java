package org.team1126.robot.util.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.team1126.lib.math.geometry.ExtPose;

public enum AutosStart {
    CENTER(3.611, 4.046, 0.0, "HUB", "HUB"),
    BUMP(3.611, 2.544, 0.0, "BUMP", "BUMP"),
    TRENCH(3.611, 0.625, 0.0, "TRENCH", "TRENCH"),
    OTHER(0.0, 0.0, 0.0, "", "");

    private final ExtPose startingPoint;
    private final String description;
    private final String abbreviation;

    AutosStart(double startingX, double startingY, double startingRot, String description, String abbreviation) {
        this.startingPoint = new ExtPose(startingX, startingY, new Rotation2d(startingRot));
        this.description = description;
        this.abbreviation = abbreviation;
    }

    public Pose2d getStartingPoint(boolean blue, boolean left) {
        return startingPoint.get(blue, left);
    }

    public String display(boolean succinct, boolean left) {
        return succinct ? abbreviation : description;
    }
}
