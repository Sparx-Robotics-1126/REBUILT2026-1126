package org.team1126.robot.util.nav.autos;

import edu.wpi.first.wpilibj.RobotBase;
import java.util.Arrays;
import org.team1126.robot.subsystems.Swerve;
import org.team1126.robot.util.Field;
import org.team1126.robot.util.nav.NorthDefaultNavigator;
import org.team1126.robot.util.nav.Waypoint;

/**
 * Utility class to help with trench navigation. This gives a wrapper around a list of
 * waypoints with their associated decels.
 */
public final class IntakeCenterAutosMap extends NorthDefaultNavigator {

    private static IntakeCenterAutosMap instance;

    public static void init(Swerve swerve) {
        instance = new IntakeCenterAutosMap(swerve);
    }

    public static IntakeCenterAutosMap get() {
        if (instance == null) {
            return null;
        }

        return instance;
    }

    /**
     * Singleton constructor
     */
    private IntakeCenterAutosMap(Swerve swerve) {
        this.swerve = swerve;

        // LEFT: x: 2.54, y: 5.230, yaw: 23.85
        // CENTER: x: 2.287, y: 4.038 yaw: 0.0
        // RIGHT: x: 3.131, y: 2.228, yaw: 51.13

        // Check for continuous input on rotation to keep it turning the right way - this is probably already set.
        double decel = RobotBase.isSimulation() ? SIMULATION_DECEL : DEFAULT_DECEL;
        waypoints = Arrays.asList(
            new Waypoint(2.975, 0.603, Math.toRadians(0.0), decel),
            new Waypoint(5.877, 0.603, Math.toRadians(0.0), decel),
            new Waypoint(Field.CENTER_X - Swerve.OFFSET, 1.2, Math.toRadians(270.0), decel),
            new Waypoint(Field.CENTER_X - Swerve.OFFSET, 5.764, Math.toRadians(270.0), decel),
            new Waypoint(Field.CENTER_X - Swerve.OFFSET, 5.764, Math.toRadians(270.0), decel),
            new Waypoint(6.815, 7.458, Math.toRadians(178.91), decel)
            // new Waypoint(2.70, 7.458, Math.toRadians(178.91), decel)
        ).toArray(new Waypoint[0]);
    }
}
