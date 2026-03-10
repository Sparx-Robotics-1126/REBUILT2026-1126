package org.team1126.robot.util.nav;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.team1126.lib.math.geometry.ExtPose;
import org.team1126.lib.math.geometry.ExtTranslation;

public final class Waypoint {

    public final String name;
    public final ExtTranslation coord;
    public final double rot;
    public final double decel;

    public Waypoint(String name, double x, double y, double rot, double decel) {
        this.coord = new ExtTranslation(new Translation2d(x, y));
        this.rot = rot;
        this.decel = decel;
        this.name = name;
    }

    public Waypoint(double x, double y, double rot, double decel) {
        this.name =
            "WAYPOINT_("
            + Double.toString(x)
            + ", "
            + Double.toString(y)
            + ")_"
            + Double.toString(rot)
            + "_"
            + Double.toString(decel);
        this.coord = new ExtTranslation(new Translation2d(x, y));
        this.rot = rot;
        this.decel = decel;
    }

    public ExtPose asPose(boolean left) {
        return new ExtPose(coord.get(left), new Rotation2d(rot));
    }
}
