package org.team1126.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.tunable.Tunables.TunableDouble;
import org.team1126.lib.util.Alliance;
import org.team1126.robot.Robot;
import org.team1126.robot.subsystems.*;
import org.team1126.robot.util.autos.AutosFlip;
import org.team1126.robot.util.autos.AutosStart;
import org.team1126.robot.util.autos.routines.BacknForth;
import org.team1126.robot.util.autos.routines.DepotPlaceholder;
import org.team1126.robot.util.autos.routines.HerdIntoSection;
import org.team1126.robot.util.autos.routines.InTheTrenches;
import org.team1126.robot.util.autos.routines.IntakeCenter;
import org.team1126.robot.util.autos.routines.GoOutpostAndToss;
import org.team1126.robot.util.autos.routines.PourMeSomeFuelTest;
import org.team1126.robot.util.autos.routines.OutpostAndCenter;
import org.team1126.robot.util.autos.routines.ShootFirstAskQuestionsLater;
import org.team1126.robot.util.autos.routines.ShootIntakeShootAtAlliance;

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

        ShootFirstAskQuestionsLater.init(robot);
        ShootIntakeShootAtAlliance.init(robot);
        OutpostAndCenter.init(robot);
        HerdIntoSection.init(robot);
        GoOutpostAndToss.init(robot);
        BacknForth.init(robot);
        DepotPlaceholder.init(robot);
        PourMeSomeFuelTest.init(robot);

        AutosFlip right = AutosFlip.RIGHT;
        AutosFlip left = AutosFlip.LEFT;
        AutosFlip noTrench = AutosFlip.NONE;
        AutosStart startTrench = AutosStart.TRENCH;
        AutosStart startCenter = AutosStart.CENTER;
        AutosStart startBump = AutosStart.BUMP;

        // Create the auto chooser
        chooser.setDefaultOption("Do nothing", Commands.none());

        // PEWPEW
        chooser.addOption(
            ShootFirstAskQuestionsLater.get().getDisplayName(startBump, right, true),
            ShootFirstAskQuestionsLater.get().action(() -> startBump, () -> right, () -> Alliance.isBlue())
        );
        chooser.addOption(
            ShootFirstAskQuestionsLater.get().getDisplayName(startCenter, right, true),
            ShootFirstAskQuestionsLater.get().action(() -> startCenter, () -> right, () -> Alliance.isBlue())
        );
        chooser.addOption(
            ShootFirstAskQuestionsLater.get().getDisplayName(startBump, left, true),
            ShootFirstAskQuestionsLater.get().action(() -> startBump, () -> left, () -> Alliance.isBlue())
        );
        chooser.addOption(
            ShootFirstAskQuestionsLater.get().getDisplayName(startCenter, left, true),
            ShootFirstAskQuestionsLater.get().action(() -> startCenter, () -> left, () -> Alliance.isBlue())
        );

        //BacknForth
        chooser.addOption(
            BacknForth.get().getDisplayName(right, true),
            BacknForth.get().action(() -> right, () -> Alliance.isBlue())
        );
        chooser.addOption(
            BacknForth.get().getDisplayName(left, true),
            BacknForth.get().action(() -> left, () -> Alliance.isBlue())
        );

        SmartDashboard.putData("autos", chooser);

        // chooser.addOption(
        //     HerdIntoSection.get().getDisplayName(right, true),
        //     HerdIntoSection.get().action(() -> right, () -> Alliance.isBlue())
        // );
        // chooser.addOption(
        //     HerdIntoSection.get().getDisplayName(left, true),
        //     HerdIntoSection.get().action(() -> left, () -> Alliance.isBlue())
        // );

        //GoOutpostAndToss
        chooser.addOption(
            GoOutpostAndToss.get().getDisplayName(startBump, right, true),
            GoOutpostAndToss.get().action(() -> startBump, () -> right, () -> Alliance.isBlue())
        );

        //PourMeSomeFuelAndDriveToNeutralZone
        chooser.addOption(
            OutpostAndCenter.get().getDisplayName(startBump, right, true),
            OutpostAndCenter.get().action(()-> startBump, () -> right, () -> Alliance.isBlue())
        );

        //DepotPlaceholder
        chooser.addOption(
            DepotPlaceholder.get().getDisplayName(startBump, left, true),
            DepotPlaceholder.get().action(() -> startBump, () -> left, () -> Alliance.isBlue())
        );
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
