package org.team1126.robot.util.nav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.BooleanSupplier;
import org.team1126.lib.math.geometry.ExtRotation;
import org.team1126.lib.math.geometry.ExtTranslation;

public final class Waypoint {

    public final String name;
    public final ExtTranslation coord;
    public final double rot;
    public final double decel;
    public final boolean limitField;

    public Waypoint(double x, double y, double rot, double decel) {
        this(x, y, rot, decel, false);
    }

    public Waypoint(double x, double y, double rot, double decel, boolean limitField) {
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
            decel,
            limitField
        );
    }

    public Waypoint(String name, double x, double y, double rot, double flipRot, double decel) {
        this(name, x, y, rot, decel, false);
    }

    public Waypoint(String name, double x, double y, double rot, double decel, boolean limitField) {
        this.coord = new ExtTranslation(new Translation2d(x, y));
        this.rot = rot;
        this.decel = decel;
        this.name = name;
        this.limitField = limitField;
    }

    public Pose2d asPose(BooleanSupplier left, BooleanSupplier blue) {
        return new Pose2d(
            coord.get(blue.getAsBoolean(), left.getAsBoolean()),
            new ExtRotation(rot).get(blue.getAsBoolean(), left.getAsBoolean())
        );
    }
}
