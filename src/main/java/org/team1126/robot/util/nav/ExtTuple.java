package org.team1126.robot.util.nav;

import edu.wpi.first.math.geometry.Translation2d;
import org.team1126.lib.math.geometry.ExtTranslation;

public final class ExtTuple {

    protected ExtTranslation waypoint;
    protected double rot;
    protected double decel;

    public ExtTuple(double x, double y, double rot, double decel) {
        this.waypoint = new ExtTranslation(new Translation2d(x, y));
        this.rot = rot;
        this.decel = decel;
    }
}
