package org.team1126.robot.util.nav;

public enum WaypointHeading {
    NORTH(0),
    SOUTH(1);

    private final int val;

    private WaypointHeading(int val) {
        this.val = val;
    }

    public int getVal() {
        return val;
    }
}
