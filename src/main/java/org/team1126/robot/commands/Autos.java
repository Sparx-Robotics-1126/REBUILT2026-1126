package org.team1126.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

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

    private static final TunableDouble intakeDecel = tunables.value("intakeDecel", 12.0);
    private static final TunableDouble intakeRotDelay = tunables.value("intakeRotDelay", 0.6);
    private static final TunableDouble avoidDecel = tunables.value("avoidDecel", 10.0);
    private static final TunableDouble avoidTol = tunables.value("avoidTol", 0.25);

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
    }

    /**
     * Returns {@code true} when the default auto is selected.
     */
    public boolean defaultSelected() {
        return chooser.defaultSelected().getAsBoolean();
    }

    // ********** Sim / Testing **********
}
