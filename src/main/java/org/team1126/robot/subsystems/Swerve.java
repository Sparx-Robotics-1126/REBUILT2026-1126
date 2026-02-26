package org.team1126.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Orchestra;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
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
import org.team1126.lib.util.Mutable;
import org.team1126.lib.util.command.GRRSubsystem;
import org.team1126.robot.Constants;
import org.team1126.robot.Constants.LowerCAN;
import org.team1126.robot.Constants.RioCAN;
import org.team1126.robot.Robot;
import org.team1126.robot.util.Field;
// import org.team1126.robot.util.Vision;

import org.team1126.robot.util.Vision;
import org.team1126.robot.util.WaypointNavigator;

/**
 * The robot's swerve drivetrain.
 */
@Logged
public final class Swerve extends GRRSubsystem {

    public static final double OFFSET = 0.2525; // 575mm from cancoder to cancoder
    // private static final double OFFSET_TO_BUMPER = 0.425; // 425mm to the edge of the bumper

    private static final TunableTable tunables = Tunables.getNested("swerve");
    private static final TunableDouble turboSpin = tunables.value("turboSpin", 8.0);
    private static final TunableDouble facingHubTol = tunables.value("facingHubTol", 1.0);

    private static final TunableTable beachTunables = tunables.getNested("beach");
    private static final TunableDouble beachSpeed = beachTunables.value("speed", 3.0);
    private static final TunableDouble beachTolerance = beachTunables.value("tolerance", 0.15);

    private static final TunableTable apfTunables = tunables.getNested("apf");
    private static final TunableDouble apfX = apfTunables.value("x", 1.14);
    private static final TunableDouble apfVel = apfTunables.value("velocity", 4.5);
    private static final TunableDouble apfLead = apfTunables.value("lead", 0.45);
    private static final TunableDouble apfLeadMult = apfTunables.value("leadMult", 0.15);
    private static final TunableDouble apfLeadAccel = apfTunables.value("leadAccel", 7.7);
    private static final TunableDouble apfLeadAccelL4 = apfTunables.value("leadAccelL4", 6.95);
    private static final TunableDouble apfScoreAccel = apfTunables.value("scoreAccel", 6.0);
    private static final TunableDouble apfScoreAccelL4 = apfTunables.value("scoreAccelL4", 3.5);
    private static final TunableDouble apfL4Ta = apfTunables.value("apfL4Ta", 4.0);
    private static final TunableDouble apfAngTolerance = apfTunables.value("angTolerance", 0.4);
    private static final TunableDouble apfSafeTolerance = apfTunables.value("safeTolerance", 0.2);
    private static final TunableDouble apfAttractStrength = apfTunables.value("attractStrength", -9.0);
    private static final TunableDouble apfAttractRange = apfTunables.value("attractRange", 2.5);

    private static final TunableTable reefAssistTunables = tunables.getNested("reefAssist");
    private static final TunableDouble reefAssistX = reefAssistTunables.value("x", 0.681);
    private static final TunableDouble reefAssistKp = reefAssistTunables.value("kP", 20.0);
    private static final TunableDouble reefAssistTolerance = reefAssistTunables.value("tolerance", 1.75);

    private static final TunableTable trenchTunables = tunables.getNested("trench");
    private static final TunableDouble trenchDecel = trenchTunables.value("deceleration", 0.3);

    private static final Orchestra orchestra = new Orchestra();
    // private static final TunableDouble climbFudge = tunables.value("climbFudge", Math.toRadians(5.0));

    private final SwerveModuleConfig frontLeft = new SwerveModuleConfig()
        .setName("frontLeft")
        .setLocation(OFFSET, OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(LowerCAN.FL_MOVE, true, orchestra))
        .setTurnMotor(SwerveMotors.talonFX(LowerCAN.FL_TURN, false, orchestra))
        .setEncoder(SwerveEncoders.cancoder(LowerCAN.FL_ENCODER, .164, false));

    private final SwerveModuleConfig frontRight = new SwerveModuleConfig()
        .setName("frontRight")
        .setLocation(OFFSET, -OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(LowerCAN.FR_MOVE, true, orchestra))
        .setTurnMotor(SwerveMotors.talonFX(LowerCAN.FR_TURN, false, orchestra))
        .setEncoder(SwerveEncoders.cancoder(LowerCAN.FR_ENCODER, 0.498, false));

    private final SwerveModuleConfig backLeft = new SwerveModuleConfig()
        .setName("backLeft")
        .setLocation(-OFFSET, OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(LowerCAN.BL_MOVE, true, orchestra))
        .setTurnMotor(SwerveMotors.talonFX(LowerCAN.BL_TURN, false, orchestra))
        .setEncoder(SwerveEncoders.cancoder(LowerCAN.BL_ENCODER, -.510, false));

    private final SwerveModuleConfig backRight = new SwerveModuleConfig()
        .setName("backRight")
        .setLocation(-OFFSET, -OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(LowerCAN.BR_MOVE, true, orchestra))
        .setTurnMotor(SwerveMotors.talonFX(LowerCAN.BR_TURN, false, orchestra))
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
    // private boolean visionEnabled = true;
    private Pose2d hubReference = Pose2d.kZero;
    private boolean seesAprilTag = false;
    private boolean changedReference = false;
    private double hubAngular = 0.0;
    private boolean shouldFaceHub = false;

    // private List<ExtPose> waypoints;

    // private PhotonCamera fuelCamera;
    private boolean fuelTargetLost;

    private boolean facingHub = false;

    private double distanceToHub = 0.0;
    private double angleToHub = 0.0;

    // private final Orchestra orchestra;

    public Swerve() {
        api = new SwerveAPI(config);
        // // Get all swerve modules
        //         var modules = api.getSwerveModules();

        //         // Add each module's drive and steer motors to the orchestra
        //         for (int i = 0; i < modules.length; i++) {
        //             var module = modules[i];
        //             TalonFX moveFX = (TalonFX) module.moveMotor;
        //             orchestra.addInstrument(module.moveMotor, 1);
        //             orchestra.addInstrument(module.getSteerMotor(), 0);
        //         }

        //  orchestra = new Orchestra();
        //         var mods = api.modules[0].moveMotor;
        // TalonFX moveFX = (TalonFX) mods;

        //        for (SwerveModule module : api.modules) {
        //     Object moveMotorAPI = module.moveMotor.getAPI();
        //     Object turnMotorAPI = module.turnMotor.getAPI();

        //     if (moveMotorAPI instanceof TalonFX) {
        //         TalonFX moveFX = (TalonFX) moveMotorAPI;
        //         // Use the TalonFX instance
        //     }
        // }
        // for (int i = 0; i < api.modules.length; i++) {
        //     var module = mods[i];
        //     orchestra.addInstrument(module.getDriveMotor(), 1);
        //     orchestra.addInstrument(module.getSteerMotor(), 0);
        // }

        vision = new Vision(Constants.AT_CAMERAS);

        apf = new PAPFController(9.0, 0.5, 0.01, true, Field.OBSTACLES);
        angularPID = new ProfiledPIDController(8.0, 0.0, 0.0, new Constraints(10.0, 26.0));
        angularPID.enableContinuousInput(-Math.PI, Math.PI);

        // fuelCamera = new PhotonCamera(Constants.OBJ_DETECTION_CAMERA_CONFIG.name());

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
        SmartDashboard.putBoolean("Is Playing Music", orchestra.isPlaying());
        // Apply vision estimates to the pose estimator.
        if (Robot.isReal()) {
            var measurements = vision.getUnreadResults(state.poseHistory, state.odometryPose, state.velocity);
            SmartDashboard.putNumber("Vision X", measurements.length);

            seesAprilTag = measurements.length > 0;
            api.addVisionMeasurements(measurements);
        }
        SmartDashboard.putNumber("Goal X", apf.getGoal().getX());
        SmartDashboard.putNumber("Goal Y", apf.getGoal().getY());

        Translation2d hubCenter = Field.HUB.get();

        Rotation2d hubAngle = new Rotation2d(
            Math.floor(
                    hubCenter.minus(state.translation).getAngle().plus(new Rotation2d(Math2.SIXTH_PI)).getRadians()
                        / Math2.THIRD_PI
                )
                * Math2.THIRD_PI
        );

        changedReference = !Math2.isNear(hubReference.getRotation(), hubAngle, 1e-6);

        hubReference = new Pose2d(hubCenter, hubAngle);

        facingHub = Math2.isNear(hubAngle, state.rotation, facingHubTol.get());

        final double deltaX = state.pose.getX() - Field.HUB.get().getX();
        final double deltaY = state.pose.getY() - Field.HUB.get().getY();

        distanceToHub = Math.hypot(deltaX, deltaY);
        angleToHub = Math.atan2(deltaY, deltaX) + Math.PI;
        // hubAngular = -angularPID.calculate(state.rotation.getRadians(), angleToHub);
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
    @NotLogged
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

    public Command driveFacingHub(DoubleSupplier x, DoubleSupplier y, DoubleSupplier angular) {
        return commandBuilder("Swerve.drive()").onExecute(() -> {
            var faceHub = Perspective.OPERATOR.toPerspectiveSpeeds(
                new ChassisSpeeds(0.0, 0.0, angularPID.calculate(state.rotation.getRadians(), angleToHub)),
                state.rotation
            );

            api.applyAssistedDriverInput(
                x.getAsDouble(),
                y.getAsDouble(),
                angular.getAsDouble(),
                faceHub,
                Perspective.OPERATOR,
                true,
                true
            );
        });
    }

    /**
     * SPIN FAST RAHHHHHHH
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     * @param angular The CCW+ angular speed to apply, from {@code [-1.0, 1.0]}.
     */
    public Command turboSpin(DoubleSupplier x, DoubleSupplier y, DoubleSupplier angular) {
        Mutable<Double> configured = new Mutable<>(0.0);
        return drive(x, y, angular)
            .beforeStarting(() -> {
                configured.value = api.config.driverAngularVel;
                api.config.driverAngularVel = turboSpin.get();
            })
            .finallyDo(() -> api.config.driverAngularVel = configured.value);
    }

    public Command aimAtHub(final DoubleSupplier maxDeceleration) {
        return commandBuilder("Swerve.aimAtHub()")
            .onInitialize(() -> angularPID.reset(state.rotation.getRadians(), state.speeds.omegaRadiansPerSecond))
            .onExecute(() -> {
                Pose2d goal = new Pose2d(state.pose.getX(), state.pose.getY(), new Rotation2d(angleToHub));
                var speeds = apf.calculate(
                    state.pose,
                    goal.getTranslation(),
                    config.velocity,
                    maxDeceleration.getAsDouble()
                );

                speeds.omegaRadiansPerSecond = angularPID.calculate(state.rotation.getRadians(), angleToHub);

                api.applySpeeds(speeds, Perspective.BLUE, true, true);
            });
    }

    public Command playMusic(String song) {
        return runEnd(
            () -> {
                if (!orchestra.isPlaying()) {
                    sing(song);
                }
            },
            orchestra::stop
        )
            .until(DriverStation::isEnabled)
            .ignoringDisable(true);
        // orchestra.loadMusic(song);
        // return run(() -> orchestra.play()).withName("Swerve.playMusic(" + song + ")");
    }

    public void sing(String song) {
        orchestra.loadMusic(song + ".chrp");
        System.out.println("Playing " + song);
        orchestra.play();
    }

    /**
     * Drives the robot to a target position using the APF, ending
     * when the robot is within a specified tolerance of the target.
     * @param target A supplier that returns the target blue origin relative field location.
     * @param deceleration A supplier that returns the deceleration for the robot to target in m/s/s.
     * @param endTolerance The tolerance in meters at which to end the command.
     */
    public Command apfDrive(Supplier<Pose2d> target, DoubleSupplier deceleration, DoubleSupplier endTolerance) {
        return apfDrive(target, deceleration).until(
            () -> target.get().getTranslation().getDistance(state.translation) < endTolerance.getAsDouble()
        );
    }

    /**
     * Drives the robot to a target position using the APF.
     * @param target A supplier that returns the target blue origin relative field location.
     * @param deceleration A supplier that returns the deceleration for the robot to target in m/s/s.
     */
    public Command apfDrive(Supplier<Pose2d> target, DoubleSupplier deceleration) {
        return commandBuilder("Swerve.apfDrive()")
            .onInitialize(() -> angularPID.reset(state.rotation.getRadians(), state.speeds.omegaRadiansPerSecond))
            .onExecute(() -> {
                Pose2d goal = target.get();
                var speeds = apf.calculate(
                    state.pose,
                    goal.getTranslation(),
                    apfVel.get(),
                    deceleration.getAsDouble(),
                    Field.OBSTACLES
                );
                speeds.omegaRadiansPerSecond = angularPID.calculate(
                    state.rotation.getRadians(),
                    goal.getRotation().getRadians()
                );

                api.applySpeeds(speeds, Perspective.BLUE, true, true);
            });
    }

    public Command driveTrench(BooleanSupplier right) {
        var waypoints = WaypointNavigator.trenching(state.pose, right.getAsBoolean());
        var waypointCommands = new ArrayList<Command>();
        if (waypoints != null && waypoints.size() > 0) {
            for (var waypoint : waypoints) {
                waypointCommands.add(driveWaypoint(waypoint.get(), trenchDecel.get()));
            }
        }
        return Commands.sequence(waypointCommands.toArray(new Command[0]));
        // CommandScheduler.getInstance().schedule(Commands.sequence(waypointCommands.toArray(new Command[0])));
    }

    private Command driveWaypoint(Pose2d goal, double deceleration) {
        return commandBuilder("Swerve.driveWaypoint()").onExecute(() -> {
            var speeds = apf.calculate(state.pose, goal.getTranslation(), apfVel.get(), deceleration);
            speeds.omegaRadiansPerSecond = angularPID.calculate(
                state.rotation.getRadians(),
                goal.getRotation().getRadians()
            );
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

    public double getHubAngular() {
        return angularPID.calculate(state.rotation.getRadians(), angleToHub);
    }

    public void applyOrchestra(Orchestra orchestra) {
        this.api.applyOrchestra(orchestra);
    }

    // private PhotonTrackedTarget getBestestTarget() {
    //     var resultsList = fuelCamera.getAllUnreadResults();
    //     if (resultsList == null || resultsList.size() > 0) {
    //         fuelTargetLost = false;
    //         return null;
    //     }
    //     var result = resultsList.get(0);
    //     PhotonTrackedTarget highestAreaTarget = null;
    //     if (result != null && result.hasTargets()) {
    //         List<PhotonTrackedTarget> targets = result.getTargets();
    //         highestAreaTarget = targets.stream().max(Comparator.comparing(PhotonTrackedTarget::getArea)).orElse(null);
    //     }
    //     if (highestAreaTarget == null) {
    //         fuelTargetLost = true;
    //     } else {
    //         fuelTargetLost = false;
    //     }
    //     return highestAreaTarget;
    // }

    // private Transform2d getTransformToFuel() {
    //     PhotonTrackedTarget bestestTarget = getBestestTarget();
    //     if (bestestTarget != null) {
    //         try {
    //             SmartDashboard.putString("cameraTargetTransform", bestestTarget.getBestCameraToTarget().toString());
    //         } catch (Exception e) {}
    //         fuelTargetLost = false;
    //         Transform2d transformToFuel = new Transform2d(
    //             bestestTarget.getPitch() + 2.5,
    //             bestestTarget.getYaw(),
    //             new Rotation2d(-bestestTarget.getYaw())
    //         );
    //         return transformToFuel; // bestestTarget.getBestCameraToTarget();
    //     } else {
    //         fuelTargetLost = true;
    //         return null;
    //     }
    // }

    // public Pose2d getFuelPose() {
    //     Transform2d transform2d = getTransformToFuel();
    //     if (transform2d != null) {
    //         Pose2d currentPose = state.pose;
    //         // Transform2d transform2d = new Transform2d(
    //         //     transform3d.getTranslation().toTranslation2d(),
    //         //     transform3d.getRotation().toRotation2d()
    //         // );
    //         Pose2d fuelPose = currentPose.plus(transform2d);
    //         fuelTargetLost = false;
    //         return fuelPose;
    //     } else {
    //         fuelTargetLost = true;
    //         return null;
    //     }
    // }
}
// @Logged
// public final class ReefAssistData {

//     private Pose2d targetPipe = Pose2d.kZero;
//     private boolean running = false;
//     private double error = 0.0;
//     private double output = 0.0;
// }
