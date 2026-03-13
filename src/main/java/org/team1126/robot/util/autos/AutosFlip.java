package org.team1126.robot.util.autos;

public enum AutosFlip {
    LEFT(true, "left trench", "LEFT"),
    RIGHT(false, "right trench", "RIGHT"),
    NONE(false, "", "");

    private final boolean flip;
    private final String description;
    private final String abbreviation;

    AutosFlip(boolean flip, String description, String abbreviation) {
        this.flip = flip;
        this.description = description;
        this.abbreviation = abbreviation;
    }

    public boolean shouldFlip() {
        return flip;
    }

    public String display(boolean succinct) {
        return succinct ? abbreviation : description;
    }
}
