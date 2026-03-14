package org.team1126.robot.util;

public enum ShootingRadius {
    L1(2.5),
    L2(3.0),
    L3(3.14159);

    private final double val;

    private ShootingRadius(double val) {
        this.val = val;
    }

    public double getVal() {
        return val;
    }
}
