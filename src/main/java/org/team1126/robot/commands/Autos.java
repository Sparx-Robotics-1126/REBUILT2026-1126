package org.team1126.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.tunable.Tunables.TunableDouble;
import org.team1126.robot.Robot;
import org.team1126.robot.subsystems.*;
import org.team1126.robot.util.autos.AutosFlip;
import org.team1126.robot.util.autos.AutosStart;
import org.team1126.robot.util.autos.routines.GrabAndShoot;
import org.team1126.robot.util.autos.routines.InTheTrenches;
import org.team1126.robot.util.autos.routines.IntakeCenter;
import org.team1126.robot.util.autos.routines.ShootFirstAskQuestionsLater;
import org.team1126.robot.util.autos.routines.SweepCenter;
import org.team1126.robot.util.nav.autos.ShootFirstAskQuestionsLaterAutosMap;

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
    private final Shooter shooter;
    private final Storage storage;
    private final Intake intake;

    private final Routines routines;

    SendableChooser<Command> chooser = new SendableChooser<>();

    public Autos(Robot robot) {
        this.robot = robot;

        lights = robot.lights;
        swerve = robot.swerve;
        shooter = robot.shooter;
        storage = robot.storage;
        intake = robot.intake;

        routines = robot.routines;

        SweepCenter.init(robot);
        IntakeCenter.init(robot);
        GrabAndShoot.init(robot);
        ShootFirstAskQuestionsLater.init(robot);
        InTheTrenches.init(robot);

        AutosFlip right = AutosFlip.RIGHT;
        AutosFlip left = AutosFlip.LEFT;
        AutosFlip noTrench = AutosFlip.NONE;
        AutosStart startLeft = AutosStart.LEFT;
        AutosStart startCenter = AutosStart.CENTER;
        AutosStart startRight = AutosStart.RIGHT;

        ShootFirstAskQuestionsLaterAutosMap.init(swerve);

        // Create the auto chooser
        chooser.setDefaultOption("Do nothing", Commands.none());
        chooser.addOption(
            InTheTrenches.get().getDisplayName(startRight, right, true),
            InTheTrenches.get().action(startRight, right)
        );
        chooser.addOption(
            InTheTrenches.get().getDisplayName(startCenter, right, true),
            InTheTrenches.get().action(startCenter, right)
        );
        chooser.addOption(
            InTheTrenches.get().getDisplayName(startLeft, left, true),
            InTheTrenches.get().action(startLeft, left)
        );
        chooser.addOption(
            InTheTrenches.get().getDisplayName(startCenter, left, true),
            InTheTrenches.get().action(startCenter, left)
        );
        chooser.addOption(
            ShootFirstAskQuestionsLater.get().getDisplayName(startRight, true),
            ShootFirstAskQuestionsLater.get().action(startRight, noTrench)
        );
        chooser.addOption(
            ShootFirstAskQuestionsLater.get().getDisplayName(startCenter, true),
            ShootFirstAskQuestionsLater.get().action(startCenter, noTrench)
        );
        chooser.addOption(
            ShootFirstAskQuestionsLater.get().getDisplayName(startLeft, true),
            ShootFirstAskQuestionsLater.get().action(startLeft, noTrench)
        );
        chooser.addOption(
            SweepCenter.get().getDisplayName(startRight, right, true),
            SweepCenter.get().action(startRight, right)
        );
        chooser.addOption(
            SweepCenter.get().getDisplayName(startCenter, right, true),
            SweepCenter.get().action(startCenter, right)
        );
        chooser.addOption(
            SweepCenter.get().getDisplayName(startCenter, left, true),
            SweepCenter.get().action(startCenter, left)
        );
        chooser.addOption(
            SweepCenter.get().getDisplayName(startLeft, left, true),
            SweepCenter.get().action(startLeft, left)
        );
        chooser.addOption(
            IntakeCenter.get().getDisplayName(startRight, right, true),
            IntakeCenter.get().action(startRight, right)
        );
        chooser.addOption(
            IntakeCenter.get().getDisplayName(startCenter, left, true),
            IntakeCenter.get().action(startCenter, left)
        );
        chooser.addOption(
            IntakeCenter.get().getDisplayName(startCenter, right, true),
            IntakeCenter.get().action(startCenter, right)
        );
        chooser.addOption(
            IntakeCenter.get().getDisplayName(startLeft, left, true),
            IntakeCenter.get().action(startLeft, left)
        );
        chooser.addOption(GrabAndShoot.get().getDisplayName(right, true), GrabAndShoot.get().action(right));
        chooser.addOption(GrabAndShoot.get().getDisplayName(left, true), GrabAndShoot.get().action(left));
        // chooser.addOption("Depot", routines.dock());
        SmartDashboard.putData("autos", chooser);
    }

    /**
     * Returns {@code true} when the default auto is selected.
     */
    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }

    public Command runSelectedAuto() {
        System.out.println("Running auto: " + chooser.getSelected().getName());
        return chooser.getSelected();
    }
}
