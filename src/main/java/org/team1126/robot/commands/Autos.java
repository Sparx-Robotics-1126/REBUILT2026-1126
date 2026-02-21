package org.team1126.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1126.lib.math.geometry.ExtPose;
import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.tunable.Tunables.TunableDouble;
import org.team1126.lib.util.command.AutoChooser;
import org.team1126.robot.Robot;
import org.team1126.robot.subsystems.Lights;
import org.team1126.robot.subsystems.Swerve;

/**
 * The Autos class declares autonomous modes, and adds them
 * to the dashboard to be selected by the drive team.
 */
@SuppressWarnings("unused")
public final class Autos {

    private static final TunableTable tunables = Tunables.getNested("autos");
    private static final TunableDouble deceleration = tunables.value("deceleration", 6.0);
    private static final TunableDouble tolerance = tunables.value("tolerance", 0.05);

    private final Robot robot;

    private final Lights lights;
    private final Swerve swerve;

    private final Routines routines;

    private final AutoChooser chooser;

    public Autos(Robot robot) {
        this.robot = robot;

        lights = robot.lights;
        swerve = robot.swerve;

        routines = robot.routines;

        // Create the auto chooser
        chooser = new AutoChooser();
        chooser.add("Drive", driveSampleLocations());
    }

    /**
     * Returns {@code true} when the default auto is selected.
     */
    public boolean defaultSelected() {
        return chooser.defaultSelected().getAsBoolean();
    }

    private Command driveSampleLocations() {
        var start = new ExtPose(2.0, 2.0, Rotation2d.kZero);
        var middle = new ExtPose(3.0, 5.0, Rotation2d.k180deg);
        var end = new ExtPose(6.0, 2.5, Rotation2d.kCW_90deg);

        return sequence(
            swerve.resetPose(start),
            swerve.apfDrive(middle, deceleration, tolerance),
            swerve.apfDrive(end, deceleration, tolerance),
            swerve.stop(false)
        );
    }

    // ********** Sim / Testing **********
}
