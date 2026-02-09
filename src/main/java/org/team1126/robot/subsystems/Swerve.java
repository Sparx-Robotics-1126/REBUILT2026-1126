package org.team1126.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.team1126.lib.logging.LoggedRobot;
import org.team1126.lib.math.FieldInfo;
import org.team1126.lib.math.Math2;
import org.team1126.lib.math.PAPFController;
import org.team1126.lib.math.PAPFController.LineObstacle;
import org.team1126.lib.math.PAPFController.Obstacle;
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
import org.team1126.lib.util.Mutable;
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

    private static final double OFFSET = 0.2875; // 575mm from cancoder to cancoder
    private static final double OFFSET_TO_BUMPER = 0.425; // 425mm to the edge of the bumper

    private static final TunableTable tunables = Tunables.getNested("swerve");
    private static final TunableDouble turboSpin = tunables.value("turboSpin", 8.0);
    private static final TunableDouble facingReefTol = tunables.value("facingReefTol", 1.0);

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
    private boolean visionEnabled = true;
    private final ReefAssistData reefAssist = new ReefAssistData();

    private Pose2d reefReference = Pose2d.kZero;
    private boolean seesAprilTag = false;
    private boolean changedReference = false;
    private boolean facingReef = false;
    private double wallDistance = 0.0;

    public Swerve() {
        api = new SwerveAPI(config);
        vision = new Vision(Constants.AT_CAMERAS);
        apf = new PAPFController(6.0, 0.25, 0.01, true, Field.OBSTACLES);
        angularPID = new ProfiledPIDController(8.0, 0.0, 0.0, new Constraints(10.0, 26.0));
        angularPID.enableContinuousInput(-Math.PI, Math.PI);

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
        if (visionEnabled) {
            var measurements = vision.getUnreadResults(state.poseHistory, state.odometryPose, state.velocity);
            SmartDashboard.putNumber("Vision X", measurements.length);

            seesAprilTag = measurements.length > 0;
            api.addVisionMeasurements(measurements);
        }

        // Calculate helpers
        // Translation2d reefCenter = Field.reef.get();
        // Translation2d reefTranslation = state.translation.minus(reefCenter);
        // Rotation2d reefAngle = new Rotation2d(
        //     Math.floor(
        //             reefCenter.minus(state.translation).getAngle().plus(new Rotation2d(Math2.SIXTH_PI)).getRadians()
        //                 / Math2.THIRD_PI
        //         )
        //         * Math2.THIRD_PI
        // );

        // Save if the reef angle has changed.
        // changedReference = !Math2.isNear(reefReference.getRotation(), reefAngle, 1e-6);

        // Save the current alliance's reef location, and the rotation
        // to the reef wall relevant to the robot's position.
        // reefReference = new Pose2d(reefCenter, reefAngle);

        // If the robot is rotated to face the reef, within an arbitrary tolerance.
        // facingReef = Math2.isNear(reefAngle, state.rotation, facingReefTol.get());
        // SmartDashboard.putBoolean("Facing Reef", facingReef);
        // Calculate the distance from the robot's center to the nearest reef wall face.
        // wallDistance = Math.max(
        //     0.0,
        //     reefAngle.rotateBy(Rotation2d.k180deg).minus(reefTranslation.getAngle()).getCos()
        //             * reefTranslation.getNorm()
        //         - Field.reefWallDist
        // );
        // SmartDashboard.putNumber("Wall Distance", wallDistance);
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
                Rotation2d originRotation = new Rotation2d(0);
                Translation2d originOffset = new Translation2d(OFFSET_TO_BUMPER, OFFSET_TO_BUMPER);
                api.resetPose(new Pose2d(originOffset, originRotation));
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

    /**
     * Drives the robot using driver input while facing the reef,
     * and "pushing" the robot to center on the selected pipe.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     * @param angular The CCW+ angular speed to apply, from {@code [-1.0, 1.0]}.
     * @param left A supplier that returns {@code true} if the robot should target
     *             the left reef pole, or {@code false} to target the right pole.
     */
    public Command driveReef(DoubleSupplier x, DoubleSupplier y, DoubleSupplier angular, BooleanSupplier left) {
        Mutable<Boolean> exitLock = new Mutable<>(false);

        return commandBuilder("Swerve.driveReef()")
            .onInitialize(() -> {
                angularPID.reset(state.rotation.getRadians(), state.speeds.omegaRadiansPerSecond);
                exitLock.value = false;
            })
            .onExecute(() -> {
                double xInput = x.getAsDouble();
                double yInput = y.getAsDouble();
                double angularInput = angular.getAsDouble();
                double norm = Math.hypot(-yInput, -xInput);
                boolean inDeadband = norm < api.config.driverVelDeadband;

                reefAssist.targetPipe = generateReefLocation(
                    reefAssistX.get(),
                    reefReference.getRotation(),
                    left.getAsBoolean()
                );

                Rotation2d robotAngle = reefAssist.targetPipe.getTranslation().minus(state.translation).getAngle();
                Rotation2d xyAngle = !inDeadband
                    ? new Rotation2d(-yInput, -xInput).rotateBy(
                          Alliance.isBlue() ? Rotation2d.kZero : Rotation2d.k180deg
                      )
                    : robotAngle;

                double stickDistance = Math.abs(
                    (Math.cos(xyAngle.getRadians()) * (reefAssist.targetPipe.getY() - state.pose.getY())
                        - Math.sin(xyAngle.getRadians()) * (reefAssist.targetPipe.getX() - state.pose.getX()))
                );

                reefAssist.running =
                    Math.abs(stickDistance) < reefAssistTolerance.get()
                    && Math2.isNear(robotAngle, xyAngle, Math2.HALF_PI)
                    && !inDeadband;

                reefAssist.error = robotAngle.minus(reefReference.getRotation()).getRadians();
                reefAssist.output = reefAssist.running ? reefAssist.error * norm * norm * reefAssistKp.get() : 0.0;

                var assist = Perspective.OPERATOR.toPerspectiveSpeeds(
                    new ChassisSpeeds(
                        0.0,
                        reefAssist.output,
                        !exitLock.value
                            ? angularPID.calculate(
                                  state.rotation.getRadians(),
                                  reefReference.getRotation().getRadians()
                              )
                            : 0.0
                    ),
                    reefReference.getRotation()
                );

                if (Math.abs(angularInput) > 1e-6) exitLock.value = true;
                api.applyAssistedDriverInput(xInput, yInput, angularInput, assist, Perspective.OPERATOR, true, true);
            })
            .onEnd(() -> reefAssist.running = false);
    }

    /**
     * Drives the robot to the reef autonomously. Targets
     * the side of the reef that the robot is closest to.
     * @param left A supplier that returns {@code true} if the robot should target
     *             the left reef pole, or {@code false} to target the right pole.
     * @param ready If the robot is ready to approach the scoring location.
     * @param l4 If the robot is scoring L4.
     */
    public Command apfDrive(BooleanSupplier left, BooleanSupplier ready, BooleanSupplier l4) {
        return apfDrive(() -> reefReference.getRotation(), left, ready, l4);
    }

    /**
     * Drives the robot to the reef autonomously.
     * @param location The reef location to drive to.
     * @param ready If the robot is ready to approach the scoring location.
     * @param l4 If the robot is scoring L4.
     */
    // This is really not needed anymore, leaving in commented while we figure out how we want to move.
    // public Command apfDrive(ReefLocation location, BooleanSupplier ready, BooleanSupplier l4) {
    //     return apfDrive(
    //         () -> Alliance.isBlue() ? location.side : location.side.rotateBy(Rotation2d.k180deg),
    //         () -> location.left,
    //         ready,
    //         l4
    //     );
    // }

    /**
     * Internal function, converts reef side to APF drive controller.
     * @param side A supplier that returns the side of the reef to target.
     * @param left A supplier that returns {@code true} if the robot should target
     *             the left reef pole, or {@code false} to target the right pole.
     * @param ready If the robot is ready to approach the scoring location.
     * @param l4 If the robot is scoring L4.
     */
    private Command apfDrive(
        Supplier<Rotation2d> side,
        BooleanSupplier left,
        BooleanSupplier ready,
        BooleanSupplier l4
    ) {
        Mutable<Pose2d> lastTarget = new Mutable<>(Pose2d.kZero);
        Mutable<Double> torqueAccel = new Mutable<>(config.torqueAccel);
        Mutable<Boolean> nowSafe = new Mutable<>(false);

        return commandBuilder("Swerve.apfDrive()")
            .onInitialize(() -> {
                angularPID.reset(state.rotation.getRadians(), state.speeds.omegaRadiansPerSecond);

                lastTarget.value = Pose2d.kZero;
                torqueAccel.value = config.torqueAccel;
                nowSafe.value = false;
            })
            .onExecute(() -> {
                Pose2d goal = reefAssist.targetPipe = generateReefLocation(apfX.get(), side.get(), left.getAsBoolean());

                Translation2d error = goal.getTranslation().minus(state.translation);
                Rotation2d robotAngle = error.getAngle();
                reefAssist.error = robotAngle.minus(side.get()).getRadians();

                if (!goal.equals(lastTarget.value)) nowSafe.value = false;
                lastTarget.value = goal;

                if (!nowSafe.value) {
                    goal = generateReefLocation(
                        apfX.get() + apfLead.get() + (apfLeadMult.get() * (error.getNorm() - apfLead.get())),
                        side.get(),
                        left.getAsBoolean()
                    );

                    if (
                        ready.getAsBoolean()
                        && state.translation.getDistance(goal.getTranslation()) * (Math.abs(reefAssist.error) / Math.PI)
                        <= apfSafeTolerance.get()
                        && Math.abs(state.rotation.minus(goal.getRotation()).getRadians()) <= apfAngTolerance.get()
                    ) {
                        if (l4.getAsBoolean()) config.torqueAccel = apfL4Ta.get();
                        nowSafe.value = true;
                    }
                }

                double deceleration = !nowSafe.value
                    ? (l4.getAsBoolean() ? apfLeadAccelL4.get() : apfLeadAccel.get())
                    : (l4.getAsBoolean() ? apfScoreAccelL4.get() : apfScoreAccel.get());

                Obstacle attract = new LineObstacle(
                    generateReefLocation(10.0, side.get(), left.getAsBoolean()).getTranslation(),
                    reefAssist.targetPipe.getTranslation(),
                    true,
                    apfAttractStrength.get(),
                    apfAttractRange.get()
                );

                var speeds = apf.calculate(state.pose, goal.getTranslation(), apfVel.get(), deceleration, attract);

                speeds.omegaRadiansPerSecond = angularPID.calculate(
                    state.rotation.getRadians(),
                    goal.getRotation().getRadians()
                );

                api.applySpeeds(speeds, Perspective.BLUE, true, true);
            })
            .onEnd(() -> config.torqueAccel = torqueAccel.value);
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
                var speeds = apf.calculate(state.pose, goal.getTranslation(), apfVel.get(), deceleration.getAsDouble());

                speeds.omegaRadiansPerSecond = angularPID.calculate(
                    state.rotation.getRadians(),
                    goal.getRotation().getRadians()
                );

                api.applySpeeds(speeds, Perspective.BLUE, true, true);
            });
    }

    /**
     * Drives the robot with the heading locked for the climb.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     */
    // public Command driveClimb(DoubleSupplier x, DoubleSupplier y) {
    //     return commandBuilder("Swerve.driveClimb()")
    //         .onInitialize(() -> angularPID.reset(state.rotation.getRadians(), state.speeds.omegaRadiansPerSecond))
    //         .onExecute(() -> {
    //             api.applyAssistedDriverInput(
    //                 x.getAsDouble(),
    //                 y.getAsDouble(),
    //                 0.0,
    //                 new ChassisSpeeds(
    //                     0.0,
    //                     0.0,
    //                     angularPID.calculate(
    //                         state.rotation.getRadians(),
    //                         Math2.HALF_PI * (Alliance.isBlue() ? 1.0 : -1.0) + climbFudge.get()
    //                     )
    //                 ),
    //                 Perspective.OPERATOR,
    //                 true,
    //                 true
    //             );
    //         });
    // }

    /**
     * Drives the modules to stop the robot from moving.
     * @param lock If the wheels should be driven to an X formation to stop the robot from being pushed.
     */
    public Command stop(boolean lock) {
        return commandBuilder("Swerve.stop(" + lock + ")").onExecute(() -> api.applyStop(lock));
    }

    // out concerns are not with the Reef this year...let's figure out another way to generate what we are focused on.
    private Pose2d generateReefLocation(double xOffset, Rotation2d side, boolean left) {
        // return new Pose2d(
        //     reefReference
        //         .getTranslation()
        //         .plus(new Translation2d(-xOffset, Field.pipeY * (left ? 1.0 : -1.0)).rotateBy(side)),
        //     side
        // );
        return Pose2d.kZero;
    }

    @Logged
    public final class ReefAssistData {

        private Pose2d targetPipe = Pose2d.kZero;
        private boolean running = false;
        private double error = 0.0;
        private double output = 0.0;
    }
}
