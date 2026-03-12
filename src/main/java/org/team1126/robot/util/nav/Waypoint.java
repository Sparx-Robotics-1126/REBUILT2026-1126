package org.team1126.robot.util.nav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.team1126.lib.math.geometry.ExtPose;
import org.team1126.lib.math.geometry.ExtTranslation;

public final class Waypoint {

    public final String name;
    public final ExtTranslation coord;
    public final double rot;
    public final double flipRot;
    public final double decel;
    public final boolean limitField;

    public Waypoint(double x, double y, double rot, double flipRot, double decel) {
        this(x, y, rot, flipRot, decel, false);
    }

    public Waypoint(double x, double y, double rot, double flipRot, double decel, boolean limitField) {
        this(
            "WAYPOINT_("
                + Double.toString(x)
                + ", "
                + Double.toString(y)
                + ")_"
                + Double.toString(rot)
                + "_"
                + Double.toString(decel),
            x,
            y,
            rot,
            flipRot,
            decel,
            limitField
        );
    }

    public Waypoint(String name, double x, double y, double rot, double flipRot, double decel) {
        this(name, x, y, rot, flipRot, decel, false);
    }

    public Waypoint(String name, double x, double y, double rot, double flipRot, double decel, boolean limitField) {
        this.coord = new ExtTranslation(new Translation2d(x, y));
        this.rot = rot;
        this.flipRot = flipRot;
        this.decel = decel;
        this.name = name;
        this.limitField = limitField;
    }

    public Pose2d asPose(boolean blue, boolean left) {
        ExtPose pose = new ExtPose(coord.get(), new Rotation2d(left ? flipRot : rot));
        return pose.get(blue, left);
    }
}
