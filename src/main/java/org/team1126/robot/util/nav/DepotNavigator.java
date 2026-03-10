package org.team1126.robot.util.nav;

import edu.wpi.first.wpilibj.RobotBase;
import java.util.Arrays;
import org.team1126.robot.subsystems.Swerve;

/**
 * Utility class to help with trench navigation. This gives a wrapper around a list of
 * waypoints with their associated decels.
 */
public final class DepotNavigator extends NorthDefaultNavigator {

    private static DepotNavigator instance;

    public static void init(Swerve swerve) {
        instance = new DepotNavigator(swerve);
    }

    public static DepotNavigator get() {
        if (instance == null) {
            return null;
        }

        return instance;
    }

    /**
     * Singleton constructor
     */
    private DepotNavigator(Swerve swerve) {
        this.swerve = swerve;

        double decel = RobotBase.isSimulation() ? SIMULATION_DECEL : DEFAULT_DECEL;
        waypoints = Arrays.asList(
            new Waypoint(2.610, 6.667, Math.toRadians(4.58), decel),
            new Waypoint(0.903, 5.867, Math.toRadians(0.0), decel)
        ).toArray(new Waypoint[0]);
    }
}
