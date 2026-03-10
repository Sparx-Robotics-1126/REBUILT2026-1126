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
public final class ShootFirstAskQuestionsLaterAutosMap extends NorthDefaultNavigator {

    // TODO: Make this tunable
    public static final double INTAKE_FACTOR = 0.75;

    private static ShootFirstAskQuestionsLaterAutosMap instance;

    public static void init(Swerve swerve) {
        instance = new ShootFirstAskQuestionsLaterAutosMap(swerve);
    }

    public static ShootFirstAskQuestionsLaterAutosMap get() {
        if (instance == null) {
            return null;
        }

        return instance;
    }

    /**
     * Singleton constructor
     */
    private ShootFirstAskQuestionsLaterAutosMap(Swerve swerve) {
        this.swerve = swerve;

        // LEFT: x: 2.54, y: 5.230, yaw: 23.85
        // CENTER: x: 2.287, y: 4.038 yaw: 0.0
        // RIGHT: x: 3.131, y: 2.228, yaw: 51.13

        // Check for continuous input on rotation to keep it turning the right way - this is probably already set.
        double decel = RobotBase.isSimulation() ? SIMULATION_DECEL : DEFAULT_DECEL;
        waypoints = Arrays.asList(
            new Waypoint((Field.CENTER_X - Swerve.OFFSET - 0.50), 1.2, Math.toRadians(90.0), decel * INTAKE_FACTOR),
            new Waypoint(
                (Field.CENTER_X - Swerve.OFFSET - 0.50),
                Field.CENTER_Y,
                Math.toRadians(90.0),
                decel * INTAKE_FACTOR
            ),
            new Waypoint(
                (Field.CENTER_X - Swerve.OFFSET - 0.50),
                Field.CENTER_Y,
                Math.toRadians(180.0),
                decel * INTAKE_FACTOR
            ),
            new Waypoint(
                (Field.CENTER_X - Swerve.OFFSET - 1.50),
                Field.CENTER_Y,
                Math.toRadians(180.0),
                decel * INTAKE_FACTOR
            ),
            new Waypoint((Field.CENTER_X - Swerve.OFFSET - 1.50), Field.CENTER_Y, Math.toRadians(0.0), decel),
            new Waypoint(5.005, 0.625, Math.toRadians(0.0), decel),
            new Waypoint(3.14, 0.625, Math.toRadians(0.0), decel),
            new Waypoint(2.849, 0.625, Math.toRadians(0.0), decel),
            new Waypoint(2.849, 2.315, Math.toRadians(36.0), decel)
        ).toArray(new Waypoint[0]);
    }
}
