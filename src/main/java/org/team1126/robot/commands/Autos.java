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
import org.team1126.robot.util.autos.routines.GrabAndShoot;
import org.team1126.robot.util.autos.routines.HerdIntoSection;
import org.team1126.robot.util.autos.routines.InTheTrenches;
import org.team1126.robot.util.autos.routines.IntakeCenter;
import org.team1126.robot.util.autos.routines.PourMeSomeFuel;
import org.team1126.robot.util.autos.routines.ShootFirstAskQuestionsLater;
import org.team1126.robot.util.autos.routines.ShootIntakeShootAtAlliance;
import org.team1126.robot.util.autos.routines.SweepCenter;

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
        ShootIntakeShootAtAlliance.init(robot);
        HerdIntoSection.init(robot);
        PourMeSomeFuel.init(robot);
        BacknForth.init(robot);
        DepotPlaceholder.init(robot);

        AutosFlip right = AutosFlip.RIGHT;
        AutosFlip left = AutosFlip.LEFT;
        AutosFlip noTrench = AutosFlip.NONE;
        AutosStart startTrench = AutosStart.TRENCH;
        AutosStart startCenter = AutosStart.CENTER;
        AutosStart startBump = AutosStart.BUMP;

        // Create the auto chooser
        chooser.setDefaultOption("Do nothing", Commands.none());
        chooser.addOption(
            InTheTrenches.get().getDisplayName(startBump, right, true),
            InTheTrenches.get().action(() -> startBump, () -> right, () -> Alliance.isBlue())
        );
        chooser.addOption(
            InTheTrenches.get().getDisplayName(startCenter, right, true),
            InTheTrenches.get().action(() -> startCenter, () -> right, () -> Alliance.isBlue())
        );
        chooser.addOption(
            InTheTrenches.get().getDisplayName(startBump, left, true),
            InTheTrenches.get().action(() -> startBump, () -> left, () -> Alliance.isBlue())
        );
        chooser.addOption(
            InTheTrenches.get().getDisplayName(startCenter, left, true),
            InTheTrenches.get().action(() -> startCenter, () -> left, () -> Alliance.isBlue())
        );

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

        // Sweep Center
        chooser.addOption(
            SweepCenter.get().getDisplayName(startCenter, right, true),
            SweepCenter.get().action(() -> startCenter, () -> right, () -> Alliance.isBlue())
        );
        chooser.addOption(
            SweepCenter.get().getDisplayName(startBump, right, true),
            SweepCenter.get().action(() -> startBump, () -> right, () -> Alliance.isBlue())
        );
        chooser.addOption(
            SweepCenter.get().getDisplayName(startCenter, left, true),
            SweepCenter.get().action(() -> startCenter, () -> left, () -> Alliance.isBlue())
        );
        chooser.addOption(
            SweepCenter.get().getDisplayName(startBump, left, true),
            SweepCenter.get().action(() -> startBump, () -> left, () -> Alliance.isBlue())
        );

        // Intake Center
        chooser.addOption(
            IntakeCenter.get().getDisplayName(startCenter, right, true),
            IntakeCenter.get().action(() -> startCenter, () -> right, () -> Alliance.isBlue())
        );
        chooser.addOption(
            IntakeCenter.get().getDisplayName(startBump, right, true),
            IntakeCenter.get().action(() -> startBump, () -> right, () -> Alliance.isBlue())
        );
        chooser.addOption(
            IntakeCenter.get().getDisplayName(startCenter, left, true),
            IntakeCenter.get().action(() -> startCenter, () -> left, () -> Alliance.isBlue())
        );
        chooser.addOption(
            IntakeCenter.get().getDisplayName(startBump, left, true),
            IntakeCenter.get().action(() -> startBump, () -> left, () -> Alliance.isBlue())
        );

        chooser.addOption(
            GrabAndShoot.get().getDisplayName(right, true),
            GrabAndShoot.get().action(() -> right, () -> Alliance.isBlue())
        );
        chooser.addOption(
            GrabAndShoot.get().getDisplayName(left, true),
            GrabAndShoot.get().action(() -> left, () -> Alliance.isBlue())
        );

        //BacknForth
        chooser.addOption(
            BacknForth.get().getDisplayName(startBump, right, true),
            BacknForth.get().action(() -> right, () -> Alliance.isBlue())
        );
        chooser.addOption(
            BacknForth.get().getDisplayName(startBump, left, true),
            BacknForth.get().action(() -> left, () -> Alliance.isBlue())
        );

        //ShootIntakeShootAtAlliance
        chooser.addOption(
            ShootIntakeShootAtAlliance.get().getDisplayName(startBump, right, true),
            ShootIntakeShootAtAlliance.get().action(() -> startBump, () -> right, () -> Alliance.isBlue())
        );
        chooser.addOption(
            ShootIntakeShootAtAlliance.get().getDisplayName(startBump, left, true),
            ShootIntakeShootAtAlliance.get().action(() -> startBump, () -> left, () -> Alliance.isBlue())
        );

        // chooser.addOption("Depot", routines.dock());
        // chooser.addOption("Depot", routines.dock());
        SmartDashboard.putData("autos", chooser);

        chooser.addOption(
            HerdIntoSection.get().getDisplayName(right, true),
            HerdIntoSection.get().action(() -> right, () -> Alliance.isBlue())
        );
        chooser.addOption(
            HerdIntoSection.get().getDisplayName(left, true),
            HerdIntoSection.get().action(() -> left, () -> Alliance.isBlue())
        );

        //PourMeSomeFuel
        chooser.addOption(
            PourMeSomeFuel.get().getDisplayName(startBump, right, true),
            PourMeSomeFuel.get().action(() -> startBump, () -> right, () -> Alliance.isBlue())
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
