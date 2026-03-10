package org.team1126.robot.util.autos;

public enum AutosFlip {
    LEFT(true, " (left)"),
    RIGHT(false, " (right)"),
    NONE(false, "");

    private final boolean flip;
    private final String description;

    AutosFlip(boolean flip, String description) {
        this.flip = flip;
        this.description = description;
    }

    public boolean shouldFlip() {
        return flip;
    }

    public String display() {
        return description;
    }
}
