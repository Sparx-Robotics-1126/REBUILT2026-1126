package org.team1126.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.team1126.robot.util.Vision.CameraConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {

    public static final double VOLTAGE = 12.0;

    public static final int DRIVER = 0;
    public static final int CO_DRIVER = 1;

    public static final CameraConfig[] CAMERAS = {
        new CameraConfig(
            "center",
            new Translation3d(0.315, -0.004, 0.14),
            new Rotation3d(0.0, Math.toRadians(15.0), Math.toRadians(-8.0))
        )
        // new CameraConfig(
        //     "left",
        //     new Translation3d(-0.330, 0.001, 0.14),
        //     new Rotation3d(0.0, Math.toRadians(15.0), Math.toRadians(37.0))
        // ),
        // new CameraConfig(
        //     "right",
        //     new Translation3d(-0.330, -0.009, 0.14),
        //     new Rotation3d(0.0, Math.toRadians(15.0), Math.toRadians(-53.0))
        // )
    };

    public static final class LowerCAN {

        public static final String LOWER_CAN = "LowerCAN";

        // Swerve
        public static final int FL_MOVE = 2;
        public static final int FL_TURN = 3;
        public static final int FR_MOVE = 4;
        public static final int FR_TURN = 5;
        public static final int BL_MOVE = 6;
        public static final int BL_TURN = 7;
        public static final int BR_MOVE = 8;
        public static final int BR_TURN = 9;

        public static final int FL_ENCODER = 10;
        public static final int FR_ENCODER = 11;
        public static final int BL_ENCODER = 12;
        public static final int BR_ENCODER = 13;

        // Elevator
        public static final int ELEVATOR_LEAD = 20;
        public static final int ELEVATOR_FOLLOW = 21;
        public static final int ELEVATOR_CANDI = 22;
    }

    public static final class RioCAN {

        // Swerve
        public static final int CANANDGYRO = 14;

        // Goose
        public static final int GOOSE_NECK = 30;
        public static final int GOOSE_BEAK = 31;
        public static final int GOOSE_CANDI = 32;

        // Intake
        public static final int INTAKE_MOTOR = 40;
        public static final int INTAKE_CANRANGE = 41;

        // Climber
        public static final int CLIMBER_MOTOR = 50;
    }

    public static final class RioIO {

        public static final int LIGHTS = 9;
    }
}
