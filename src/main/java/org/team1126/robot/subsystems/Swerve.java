package org.team1126.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Comparator;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team1126.lib.logging.LoggedRobot;
import org.team1126.lib.math.FieldInfo;
import org.team1126.lib.math.Math2;
import org.team1126.lib.math.PAPFController;
import org.team1126.lib.swerve.Perspective;
import org.team1126.lib.swerve.SwerveAPI;
import org.team1126.lib.swerve.SwerveState;
import org.team1126.lib.swerve.config.SwerveConfig;
import org.team1126.lib.swerve.config.SwerveModuleConfig;
import org.team1126.lib.swerve.hardware.SwerveEncoders;
import org.team1126.lib.swerve.hardware.SwerveIMUs;
import org.team1126.lib.swerve.hardware.SwerveMotors;
import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.tunable.Tunables.TunableDouble;
import org.team1126.lib.util.Alliance;
import org.team1126.lib.util.command.GRRSubsystem;
import org.team1126.robot.Constants;
import org.team1126.robot.Constants.LowerCAN;
import org.team1126.robot.Constants.RioCAN;
import org.team1126.robot.util.Field;
// import org.team1126.robot.util.Vision;

import org.team1126.robot.util.Vision;

/**
 * The robot's swerve drivetrain.
 */
@Logged
public final class Swerve extends GRRSubsystem {

    public static final double OFFSET = 0.2525; // 575mm from cancoder to cancoder
    // private static final double OFFSET_TO_BUMPER = 0.425; // 425mm to the edge of the bumper

    private static final TunableTable tunables = Tunables.getNested("swerve");

    private static final TunableTable beachTunables = tunables.getNested("beach");
    private static final TunableDouble beachSpeed = beachTunables.value("speed", 3.0);
    private static final TunableDouble beachTolerance = beachTunables.value("tolerance", 0.15);

    private static final TunableDouble aimAtHubTolerance = tunables.value("aimAtHubTolerance", 0.0);
    private static final TunableDouble zeroVelocityTolerance = tunables.value("zeroVelocityTolerance", 0.0);

    // private static final TunableDouble climbFudge = tunables.value("climbFudge", Math.toRadians(5.0));

    private final SwerveModuleConfig frontLeft = new SwerveModuleConfig()
        .setName("frontLeft")
        .setLocation(OFFSET, OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(LowerCAN.FL_MOVE, true))
        .setTurnMotor(SwerveMotors.talonFX(LowerCAN.FL_TURN, false))
        .setEncoder(SwerveEncoders.cancoder(LowerCAN.FL_ENCODER, .164, false));

    private final SwerveModuleConfig frontRight = new SwerveModuleConfig()
        .setName("frontRight")
        .setLocation(OFFSET, -OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(LowerCAN.FR_MOVE, true))
        .setTurnMotor(SwerveMotors.talonFX(LowerCAN.FR_TURN, false))
        .setEncoder(SwerveEncoders.cancoder(LowerCAN.FR_ENCODER, 0.498, false));

    private final SwerveModuleConfig backLeft = new SwerveModuleConfig()
        .setName("backLeft")
        .setLocation(-OFFSET, OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(LowerCAN.BL_MOVE, true))
        .setTurnMotor(SwerveMotors.talonFX(LowerCAN.BL_TURN, false))
        .setEncoder(SwerveEncoders.cancoder(LowerCAN.BL_ENCODER, -.510, false));

    private final SwerveModuleConfig backRight = new SwerveModuleConfig()
        .setName("backRight")
        .setLocation(-OFFSET, -OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(LowerCAN.BR_MOVE, true))
        .setTurnMotor(SwerveMotors.talonFX(LowerCAN.BR_TURN, false))
        .setEncoder(SwerveEncoders.cancoder(LowerCAN.BR_ENCODER, 0.347, false));

    private final SwerveConfig config = new SwerveConfig()
        .setTimings(LoggedRobot.DEFAULT_PERIOD, 0.004, 0.02, 0.02)
        //   Odometry: how fast the odometry updates, lower if we need lower CAN utilization.
        //             Discretization: Should not be changed.
        .setMovePID(0.325, 0.0, 0.0)
        .setMoveFF(0.0, 0.128)
        .setTurnPID(75, 0.0, 0.1)
        .setBrakeMode(true, true)
        .setLimits(4.5, 0.01, 17.0, 14.0, 36.0)
        //   Velocity: the max speed the MOTORS are able to go. please don't change this.
        //             VelDeadband: how much deadband is needed on the controller to start moving.
        //             SlipAccel: the amount of acceleration the robot can do. the higher the number, the snappier the chnage in direction will be
        //             TorqueAccel: the amount to correct for rotating while at full speed. the higher the number, the more correction.
        .setDriverProfile(2.5, 1.5, 0.15, 5.4, 2.0, 0.05)
        //  Vel: the max velocity the HUMAN PLAYER can give to the robot.
        //             velExp: the exponential given to the joystick to allow for more sensitive control
        //             velDeadband: self-explanitory
        //             angularVel: limits the angular velocity so that we dont overuse motors
        //             angularVelDeadband: also self-explanatory
        .setPowerProperties(Constants.VOLTAGE, 100.0, 80.0, 60.0, 60.0)
        .setMechanicalProperties(5.27, (287 / 11), Units.inchesToMeters(3.85))
        .setOdometryStd(0.1, 0.1, 0.05)
        .setIMU(SwerveIMUs.pigeon2(RioCAN.CANANDGYRO))
        .setPhoenixFeatures(new CANBus(LowerCAN.LOWER_CAN), false, false, false)
        .setModules(frontLeft, frontRight, backLeft, backRight);

    @NotLogged
    private final SwerveState state;

    private final SwerveAPI api;
    private final Vision vision;
    private final PAPFController apf;
    private final ProfiledPIDController angularPID;

    private double distanceToHub = 0.0;
    private double angleToHub = 0.0;

    private boolean seesAprilTag = false;
    private boolean changedReference = false;

    private PhotonCamera fuelCamera;
    private boolean fuelTargetLost;

    public Swerve() {
        api = new SwerveAPI(config);
        vision = new Vision(Constants.AT_CAMERAS);
        apf = new PAPFController(6.0, 0.25, 0.01, true, Field.OBSTACLES);
        angularPID = new ProfiledPIDController(8.0, 0.0, 0.0, new Constraints(10.0, 26.0));
        angularPID.enableContinuousInput(-Math.PI, Math.PI);

        fuelCamera = new PhotonCamera(Constants.OBJ_DETECTION_CAMERA_CONFIG.name());
        fuelTargetLost = true;

        state = api.state;

        tunables.add("api", api);
        tunables.add("apf", apf);
        tunables.add("angularPID", angularPID);
    }

    @Override
    public void periodic() {
        // Refresh the swerve API.
        api.refresh();

        // Apply vision estimates to the pose estimator.
        final var measurements = vision.getUnreadResults(state.poseHistory, state.odometryPose, state.velocity);
        SmartDashboard.putNumber("Vision X", measurements.length);

        seesAprilTag = measurements.length > 0;
        api.addVisionMeasurements(measurements);

        final double deltaX = state.pose.getX() - Field.HUB_CENTER_X;
        final double deltaY = state.pose.getY() - Field.HUB_CENTER_Y;

        distanceToHub = Math.hypot(deltaX, deltaY);
        angleToHub = Math.atan2(deltaY, deltaX);

        SmartDashboard.putNumber("Goal X", apf.getGoal().getX());
        SmartDashboard.putNumber("Goal Y", apf.getGoal().getY());

        SmartDashboard.putNumber("D2Hub", distanceToHub);
        SmartDashboard.putNumber("A2Hub", angleToHub);
    }

    /**
     * Returns the current blue origin relative pose of the robot.
     */
    @NotLogged
    public Pose2d getPose() {
        return state.pose;
    }

    /**
     * Returns the directionless measured velocity of the robot, in m/s.
     */
    @NotLogged
    public double getVelocity() {
        return state.velocity;
    }

    /**
     * Remove @NotLogged for debugging
     */
    // @NotLogged
    public List<Pose2d> apfVisualization() {
        return apf.visualizeField(40, 1.0, FieldInfo.length(), FieldInfo.width());
    }

    /**
     * Returns {@code true} if an AprilTag has been seen since the last robot loop.
     */
    @NotLogged
    public boolean seesAprilTag() {
        return seesAprilTag;
    }

    /**
     * Returns true if the reef angle has changed.
     */
    @NotLogged
    public boolean changedReference() {
        return changedReference;
    }

    /**
     * Tares the rotation of the robot. Useful for
     * fixing an out of sync or drifting IMU.
     */
    public Command tareRotation() {
        return commandBuilder("Swerve.tareRotation()")
            .onInitialize(() -> {
                api.tareRotation(Perspective.OPERATOR);
                vision.reset();
            })
            .isFinished(true)
            .ignoringDisable(true);
    }

    /**
     * Resets the pose of the robot, inherently seeding field-relative movement.
     * @param pose A supplier that returns the new blue origin
     *             relative pose to apply to the pose estimator.
     */
    public Command resetPose(Supplier<Pose2d> pose) {
        return commandBuilder("Swerve.resetPose()")
            .onInitialize(() -> {
                api.resetPose(pose.get());
                vision.reset();
            })
            .isFinished(true)
            .ignoringDisable(true);
    }

    /**
     * Hard reset of odometry to origin relative to blue origin. This will assume the robot's back
     * right corner is on 0,0.
     */
    public Command resetOdometry() {
        return commandBuilder("Swerve.resetOdometry()")
            .onInitialize(() -> {
                // Rotation2d originRotation = new Rotation2d(0);
                // Translation2d originOffset = new Translation2d(OFFSET_TO_BUMPER, OFFSET_TO_BUMPER);
                // api.resetPose(new Pose2d(originOffset, originRotation));
            })
            .isFinished(true)
            .ignoringDisable(true);
    }

    /**
     * Drives the robot using driver input.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     * @param angular The CCW+ angular speed to apply, from {@code [-1.0, 1.0]}.
     */
    public Command drive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier angular) {
        return commandBuilder("Swerve.drive()").onExecute(() -> {
            double pitch = state.pitch.getRadians();
            double roll = state.roll.getRadians();

            var antiBeach = Perspective.OPERATOR.toPerspectiveSpeeds(
                new ChassisSpeeds(
                    Math.abs(pitch) > beachTolerance.get() ? Math.copySign(beachSpeed.get(), pitch) : 0.0,
                    Math.abs(roll) > beachTolerance.get() ? Math.copySign(beachSpeed.get(), -roll) : 0.0,
                    0.0
                ),
                state.rotation
            );

            api.applyAssistedDriverInput(
                x.getAsDouble(),
                y.getAsDouble(),
                angular.getAsDouble(),
                antiBeach,
                Perspective.OPERATOR,
                true,
                true
            );
        });
    }

    private PhotonTrackedTarget getBestestTarget() {
        var resultsList = fuelCamera.getAllUnreadResults();
        if (resultsList == null || resultsList.size() > 0) {
            fuelTargetLost = false;
            return null;
        }
        var result = resultsList.get(0);
        PhotonTrackedTarget highestAreaTarget = null;
        if (result != null && result.hasTargets()) {
            List<PhotonTrackedTarget> targets = result.getTargets();
            highestAreaTarget = targets.stream().max(Comparator.comparing(PhotonTrackedTarget::getArea)).orElse(null);
        }
        if (highestAreaTarget == null) {
            fuelTargetLost = true;
        } else {
            fuelTargetLost = false;
        }
        return highestAreaTarget;
    }

    private Transform2d getTransformToFuel() {
        PhotonTrackedTarget bestestTarget = getBestestTarget();
        if (bestestTarget != null) {
            try {
                SmartDashboard.putString("cameraTargetTransform", bestestTarget.getBestCameraToTarget().toString());
            } catch (Exception e) {}
            fuelTargetLost = false;
            Transform2d transformToFuel = new Transform2d(
                bestestTarget.getPitch() + 2.5,
                bestestTarget.getYaw(),
                new Rotation2d(-bestestTarget.getYaw())
            );
            return transformToFuel; // bestestTarget.getBestCameraToTarget();
        } else {
            fuelTargetLost = true;
            return null;
        }
    }

    public Pose2d getFuelPose() {
        Transform2d transform2d = getTransformToFuel();
        if (transform2d != null) {
            Pose2d currentPose = state.pose;
            // Transform2d transform2d = new Transform2d(
            //     transform3d.getTranslation().toTranslation2d(),
            //     transform3d.getRotation().toRotation2d()
            // );
            Pose2d fuelPose = currentPose.plus(transform2d);
            fuelTargetLost = false;
            return fuelPose;
        } else {
            fuelTargetLost = true;
            return null;
        }
    }

    /**
     * Drives the robot to a target position using the P-APF, until the
     * robot is positioned within a specified tolerance of the target.
     * @param goal A supplier that returns the target blue-origin relative field location.
     * @param maxDeceleration A supplier that returns the desired deceleration rate of the robot, in m/s/s.
     * @param endTolerance The tolerance in meters at which to end the command.
     */
    public Command apfDrive(Supplier<Pose2d> goal, DoubleSupplier maxDeceleration, DoubleSupplier endTolerance) {
        return apfDrive(goal, maxDeceleration)
            .until(() -> Math2.isNear(goal.get().getTranslation(), state.translation, endTolerance.getAsDouble()))
            .withName("Swerve.apfDrive()");
    }

    /**
     * Drives the robot to a target position using the P-APF. This command does not end.
     * @param goal A supplier that returns the target blue-origin relative field location.
     * @param maxDeceleration A supplier that returns the desired deceleration rate of the robot, in m/s/s.
     */
    public Command apfDrive(Supplier<Pose2d> goal, DoubleSupplier maxDeceleration) {
        return commandBuilder("Swerve.apfDrive()")
            .onInitialize(() -> angularPID.reset(state.rotation.getRadians(), state.speeds.omegaRadiansPerSecond))
            .onExecute(() -> {
                Pose2d next = goal.get();
                var speeds = apf.calculate(
                    state.pose,
                    next.getTranslation(),
                    config.velocity,
                    maxDeceleration.getAsDouble()
                );

                speeds.omegaRadiansPerSecond = angularPID.calculate(
                    state.rotation.getRadians(),
                    next.getRotation().getRadians()
                );

                api.applySpeeds(speeds, Perspective.BLUE, true, true);
            });
    }

    /**
     * Drives the robot to a target position using the P-APF while aiming at the hub. This command does not end.
     * @param goal A supplier that returns the target blue-origin relative field location.
     * @param maxDeceleration A supplier that returns the desired deceleration rate of the robot, in m/s/s.
     */
    public Command aimAtHub(final Supplier<Translation2d> goal, final DoubleSupplier maxDeceleration) {
        return commandBuilder("Swerve.aimAtHub()")
            .onInitialize(() -> angularPID.reset(state.rotation.getRadians(), state.speeds.omegaRadiansPerSecond))
            .onExecute(() -> {
                var speeds = apf.calculate(state.pose, goal.get(), config.velocity, maxDeceleration.getAsDouble());

                speeds.omegaRadiansPerSecond = angularPID.calculate(state.rotation.getRadians(), angleToHub);
            });
    }

    /*
     * Drives the robot to a target position using the P-APF while aiming in the direction of our alliance zone (0 or PI radians). This command does not end.
     * @param goal A supplier that returns the target blue-origin relative field location.
     * @param maxDeceleration A supplier that returns the desired deceleration rate of the robot, in m/s/s.
     */
    public Command aimAtOurZone(final DoubleSupplier x, final DoubleSupplier y) {
        final double target = Alliance.isBlue() ? Math.PI : 0.0;

        return commandBuilder("Swerve.aimAtOurZone()")
            .onInitialize(() -> angularPID.reset(state.rotation.getRadians(), state.speeds.omegaRadiansPerSecond))
            .onExecute(() -> {
                final double angularVelocity = angularPID.calculate(state.rotation.getRadians(), target);

                var speeds = api.calculateDriverSpeeds(distanceToHub, angleToHub, angularVelocity);

                api.applySpeeds(speeds, Perspective.BLUE, true, true);
            });
    }

    /**
     * Drives the modules to stop the robot from moving.
     * @param lock If the wheels should be driven to an X formation to stop the robot from being pushed.
     */
    public Command stop(boolean lock) {
        return commandBuilder("Swerve.stop(" + lock + ")").onExecute(() -> api.applyStop(lock));
    }

    /*
     * Checks if the robot is aiming at the hub and not rotating within a tolerance.
     * @return True if the robot is aiming at the hub, false otherwise.
     */
    public boolean aimingAtHub() {
        final double angleDifference = Math.abs(state.rotation.getRadians() - angleToHub);
        final double angleTolerance = aimAtHubTolerance.get();

        return (
            (angleDifference < angleDifference || angleDifference > Math2.TWO_PI - angleTolerance)
            && Math.abs(state.speeds.omegaRadiansPerSecond) < zeroVelocityTolerance.get()
        );
    }

    /**
     * Checks if the origin of the robot is in our alliance's zone (the blue zone if we are on the blue alliance, and the red zone if we are on the red alliance).
     * @return True if we are in our zone, false otherwise.
     */
    public boolean inOurZone() {
        return Alliance.isBlue() ? Field.BLUE_ZONE > state.pose.getX() : Field.RED_ZONE < state.pose.getX();
    }

    /**
     * Checks if the origin of the robot is in the neutral zone (between the blue zone and the red zone).
     * @return True if we are in the neutral zone, false otherwise.
     */
    public boolean inNeutralZone() {
        final double x = state.pose.getX();
        return Field.BLUE_ZONE <= x && Field.RED_ZONE >= x;
    }

    /**
     * Checks if the origin of the robot is in the opposing alliance's zone (the red zone if we are on the blue alliance, and the blue zone if we are on the red alliance).
     * @return True if we are in the opposing alliance's zone, false otherwise.
     */
    public boolean inTheirZone() {
        return Alliance.isBlue() ? Field.RED_ZONE < state.pose.getX() : Field.BLUE_ZONE > state.pose.getX();
    }

    /**
     * Returns the distance from the origin of our robot to the center of the hub in meters.
     * @return The distance from the origin of our robot to the center of the hub, recalculated every code cycle.
     */
    @NotLogged
    public double distanceToHub() {
        return distanceToHub;
    }

    /**
     * Returns the angle from the origin of our robot to the center of the hub in radians.
     * @return The angle from the origin of our robot to the center of the hub, recalculated every code cycle.
     */
    @NotLogged
    public double angleToHub() {
        return angleToHub;
    }
}
