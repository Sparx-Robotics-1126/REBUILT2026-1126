package org.team1126.robot.util.nav;

import edu.wpi.first.wpilibj.RobotBase;
import java.util.Arrays;
import org.team1126.robot.subsystems.Swerve;

/**
 * Utility class to help with trench navigation. This gives a wrapper around a list of
 * waypoints with their associated decels.
 */
public final class TrenchNavigator extends NorthDefaultNavigator {

    private static TrenchNavigator instance;

    public static void init(Swerve swerve) {
        instance = new TrenchNavigator(swerve);
    }

    public static TrenchNavigator get() {
        if (instance == null) {
            return null;
        }

        return instance;
    }

    /**
     * Singleton constructor
     */
    private TrenchNavigator(Swerve swerve) {
        this.swerve = swerve;

        double decel = RobotBase.isSimulation() ? SIMULATION_DECEL : DEFAULT_DECEL;
        waypoints = Arrays.asList(
            new Waypoint(2.397, 2.252, Math.toRadians(35.13), decel),
            new Waypoint(2.975, 0.603, Math.toRadians(0.0), decel),
            new Waypoint(5.877, 0.603, Math.toRadians(0.0), decel),
            new Waypoint(6.927, 2.028, Math.toRadians(34.69), decel)
        ).toArray(new Waypoint[0]);
    }
}
