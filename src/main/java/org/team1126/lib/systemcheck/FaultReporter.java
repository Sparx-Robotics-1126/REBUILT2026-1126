package org.team1126.lib.systemcheck;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.team1126.lib.systemcheck.selfcheck.SelfChecking;
import org.team1126.lib.systemcheck.selfcheck.SelfCheckingCANBus;
import org.team1126.lib.systemcheck.selfcheck.SelfCheckingCANcoder;
import org.team1126.lib.systemcheck.selfcheck.SelfCheckingCanandgyro;
import org.team1126.lib.systemcheck.selfcheck.SelfCheckingCanandmag;
import org.team1126.lib.systemcheck.selfcheck.SelfCheckingPhotonCamera;
import org.team1126.lib.systemcheck.selfcheck.SelfCheckingPigeon2;
import org.team1126.lib.systemcheck.selfcheck.SelfCheckingSpark;
import org.team1126.lib.systemcheck.selfcheck.SelfCheckingTalonFX;

/**
 * Collects faults from the robot's hardware and system checks and publishes
 * them to the dashboard. Based on the FaultReporter from FRC 3061 Huskie
 * Robotics (itself derived from FRC 3015 Ranger Robotics' AdvancedSubsystem),
 * rewritten for SparX 1126's GRRBase code.
 *
 * <p>There are two kinds of checking:
 * <ul>
 *   <li><b>Device self-checks</b> (always on): every device registered with
 *       {@code registerHardware(...)} is polled about once a second while the
 *       robot is disabled, plus at the end of every system check. These catch
 *       unplugged CAN, brownouts, overheating, bad magnets, reboots, etc.
 *       Nothing moves.</li>
 *   <li><b>System checks</b> (on demand): each subsystem registers a command
 *       with {@link #registerSystemCheck} that actually moves the mechanism
 *       and verifies it behaves (e.g. the flywheel spins up, every swerve
 *       module tracks its target). The command reports problems with
 *       {@link #addFault}. Run them from the dashboard with the robot
 *       enabled in <b>Test</b> mode.</li>
 * </ul>
 *
 * <p>Everything is published to NetworkTables under {@code SmartDashboard/SystemStatus}:
 * <pre>
 *   SystemStatus/AllOK                    boolean, true if no subsystem has an error
 *   SystemStatus/RunAllChecks             button, runs every system check in order
 *   SystemStatus/ClearAllFaults           button, clears faults (including sticky faults on devices)
 *   SystemStatus/&lt;Subsystem&gt;/Status       "OK", "WARNING" or "ERROR"
 *   SystemStatus/&lt;Subsystem&gt;/SystemOK     boolean
 *   SystemStatus/&lt;Subsystem&gt;/CheckRan     boolean, true once the system check has completed
 *   SystemStatus/&lt;Subsystem&gt;/Faults       string[] of fault descriptions
 *   SystemStatus/&lt;Subsystem&gt;/SystemCheck  button, runs that subsystem's system check
 * </pre>
 * Each fault is also raised as a WPILib {@link Alert}, so it shows in Elastic's
 * alerts widget and in AdvantageScope.
 */
public final class FaultReporter {

    private static final String TABLE = "SystemStatus/";
    private static final double POLL_PERIOD_SECONDS = 1.0;

    private static FaultReporter instance;

    /**
     * Returns the FaultReporter. The first call starts the background polling.
     */
    public static FaultReporter getInstance() {
        if (instance == null) {
            instance = new FaultReporter();
        }
        return instance;
    }

    private static final class SubsystemEntry {

        private final String name;
        private final List<SelfChecking> hardware = new ArrayList<>();
        /** Latest results from polling the hardware. Replaced on every poll. */
        private final Set<SubsystemFault> hardwareFaults = new LinkedHashSet<>();
        /** Problems found by the system check. Kept until the check runs again or faults are cleared. */
        private final Set<SubsystemFault> checkFaults = new LinkedHashSet<>();
        /** One alert per currently reported fault. */
        private final Map<SubsystemFault, Alert> alerts = new LinkedHashMap<>();
        private Supplier<Command> systemCheckFactory = null;
        private boolean checkRan = false;

        private SubsystemEntry(String name) {
            this.name = name;
        }

        private Set<SubsystemFault> allFaults() {
            Set<SubsystemFault> all = new LinkedHashSet<>(hardwareFaults);
            all.addAll(checkFaults);
            return all;
        }
    }

    private final Map<String, SubsystemEntry> subsystems = new LinkedHashMap<>();
    private double lastPollTime = 0.0;

    private FaultReporter() {
        // Poll from the scheduler's event loop rather than from a scheduled
        // command, so a CommandScheduler.cancelAll() can't stop it.
        CommandScheduler.getInstance()
            .getDefaultButtonLoop()
            .bind(() -> {
                double now = Timer.getFPGATimestamp();
                if (now - lastPollTime >= POLL_PERIOD_SECONDS) {
                    lastPollTime = now;
                    pollWhenDisabled();
                }
            });
    }

    // ------------------------------------------------------------------
    // Registering hardware
    // ------------------------------------------------------------------

    /**
     * Registers any self-checking hardware with a subsystem.
     * @param subsystemName The subsystem the hardware belongs to, e.g. "Shooter".
     * @param device The self-check for the device.
     */
    public void registerHardware(String subsystemName, SelfChecking device) {
        entry(subsystemName).hardware.add(device);
    }

    /** Registers a TalonFX. */
    public void registerHardware(String subsystemName, String label, TalonFX motor) {
        registerHardware(subsystemName, new SelfCheckingTalonFX(label, motor));
    }

    /** Registers a CANcoder. */
    public void registerHardware(String subsystemName, String label, CANcoder encoder) {
        registerHardware(subsystemName, new SelfCheckingCANcoder(label, encoder));
    }

    /** Registers a Pigeon2. */
    public void registerHardware(String subsystemName, String label, Pigeon2 pigeon) {
        registerHardware(subsystemName, new SelfCheckingPigeon2(label, pigeon));
    }

    /** Registers a SparkMax or SparkFlex. */
    public void registerHardware(String subsystemName, String label, SparkBase spark) {
        registerHardware(subsystemName, new SelfCheckingSpark(label, spark));
    }

    /** Registers a Redux Canandgyro. */
    public void registerHardware(String subsystemName, String label, Canandgyro gyro) {
        registerHardware(subsystemName, new SelfCheckingCanandgyro(label, gyro));
    }

    /** Registers a Redux Canandmag. */
    public void registerHardware(String subsystemName, String label, Canandmag encoder) {
        registerHardware(subsystemName, new SelfCheckingCanandmag(label, encoder));
    }

    /** Registers a PhotonVision camera (pass the PhotonCamera the robot already uses). */
    public void registerHardware(String subsystemName, PhotonCamera camera) {
        registerHardware(subsystemName, new SelfCheckingPhotonCamera(camera));
    }

    /** Registers a CAN bus (roboRIO bus or a CANivore). */
    public void registerHardware(String subsystemName, String label, CANBus bus) {
        registerHardware(subsystemName, new SelfCheckingCANBus(label, bus));
    }

    /**
     * Registers a device whose type is only known at runtime, such as the
     * objects returned by GRRBase's {@code SwerveMotor.getAPI()}. Supported
     * types are TalonFX, CANcoder, Pigeon2, SparkMax/SparkFlex, Canandgyro and
     * Canandmag. Anything else is reported as a warning so it isn't silently skipped.
     * @param subsystemName The subsystem the hardware belongs to.
     * @param label The name shown in fault messages.
     * @param device The vendor device object.
     */
    public void registerDevice(String subsystemName, String label, Object device) {
        if (device instanceof TalonFX talonFX) {
            registerHardware(subsystemName, label, talonFX);
        } else if (device instanceof CANcoder cancoder) {
            registerHardware(subsystemName, label, cancoder);
        } else if (device instanceof Pigeon2 pigeon2) {
            registerHardware(subsystemName, label, pigeon2);
        } else if (device instanceof SparkBase spark) {
            registerHardware(subsystemName, label, spark);
        } else if (device instanceof Canandgyro canandgyro) {
            registerHardware(subsystemName, label, canandgyro);
        } else if (device instanceof Canandmag canandmag) {
            registerHardware(subsystemName, label, canandmag);
        } else {
            String type = device == null ? "null" : device.getClass().getSimpleName();
            SubsystemFault unsupported = SubsystemFault.warning(
                "[" + label + "] device type " + type + " is not supported by the system check"
            );
            registerHardware(subsystemName, () -> List.of(unsupported));
        }
    }

    // ------------------------------------------------------------------
    // System checks
    // ------------------------------------------------------------------

    /**
     * Registers a subsystem's system check and adds a button for it on the
     * dashboard at {@code SystemStatus/<subsystemName>/SystemCheck}.
     *
     * <p>Pass a factory (e.g. {@code this::systemCheck}), not a single command,
     * because the check is used both by its own button and by "RunAllChecks".
     *
     * <p>Checks only run with the robot enabled in <b>Test</b> mode, and never
     * when connected to the FMS, so a stray dashboard click can't move the
     * robot during a match. Pressing the button in any other mode adds a
     * warning telling the user to enable Test mode.
     *
     * @param subsystemName The subsystem name, e.g. "Shooter".
     * @param checkFactory Creates the system check command. It should call
     *                     {@link #addFault} for anything wrong.
     */
    public void registerSystemCheck(String subsystemName, Supplier<Command> checkFactory) {
        SubsystemEntry entry = entry(subsystemName);
        entry.systemCheckFactory = checkFactory;
        SmartDashboard.putData(
            TABLE + subsystemName + "/SystemCheck",
            dashboardButton(subsystemName + "Check", () -> wrapSystemCheck(entry), List.of(entry))
        );
        publish(entry);
    }

    /**
     * Records a problem found by a system check. Call this from inside a
     * subsystem's system check command.
     * @param subsystemName The subsystem name.
     * @param description What went wrong, in plain words.
     * @param isWarning {@code true} for a warning, {@code false} for an error.
     */
    public void addFault(String subsystemName, String description, boolean isWarning) {
        SubsystemEntry entry = entry(subsystemName);
        entry.checkFaults.add(new SubsystemFault(description, isWarning));
        publish(entry);
    }

    /**
     * Records an error found by a system check.
     * @param subsystemName The subsystem name.
     * @param description What went wrong, in plain words.
     */
    public void addFault(String subsystemName, String description) {
        addFault(subsystemName, description, false);
    }

    /**
     * Returns every fault currently reported for a subsystem.
     * @param subsystemName The subsystem name.
     */
    public List<SubsystemFault> getFaults(String subsystemName) {
        return List.copyOf(entry(subsystemName).allFaults());
    }

    /**
     * Returns the status of a subsystem (OK, WARNING or ERROR).
     * @param subsystemName The subsystem name.
     */
    public SystemStatus getStatus(String subsystemName) {
        return SystemStatus.of(entry(subsystemName).allFaults());
    }

    /**
     * Returns {@code true} if no registered subsystem has an error.
     */
    public boolean allOK() {
        for (SubsystemEntry entry : subsystems.values()) {
            if (SystemStatus.of(entry.allFaults()) == SystemStatus.ERROR) return false;
        }
        return true;
    }

    /**
     * Creates a command that runs every registered system check, one after
     * another, in the order they were registered.
     */
    public Command getRunAllChecksCommand() {
        List<Command> checks = new ArrayList<>();
        for (SubsystemEntry entry : subsystems.values()) {
            if (entry.systemCheckFactory != null) checks.add(wrapSystemCheck(entry));
        }
        return Commands.sequence(checks.toArray(Command[]::new))
            .onlyIf(DriverStation::isTest)
            .unless(DriverStation::isFMSAttached)
            .withName("RunAllChecks");
    }

    /**
     * Creates a command that clears every fault, including sticky faults
     * stored on the devices themselves, then polls the hardware again.
     * Works while disabled.
     */
    public Command getClearAllFaultsCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                for (SubsystemEntry entry : subsystems.values()) {
                    for (SelfChecking device : entry.hardware) {
                        device.clearStickyFaults();
                    }
                    entry.checkFaults.clear();
                    entry.checkRan = false;
                }
            }),
            // Give the devices time to send their updated (cleared) fault flags.
            Commands.waitSeconds(0.5),
            Commands.runOnce(this::pollAll)
        )
            .ignoringDisable(true)
            .withName("ClearAllFaults");
    }

    /**
     * Puts the "RunAllChecks" and "ClearAllFaults" buttons on the dashboard.
     * Call once in the Robot constructor, after every subsystem has registered.
     */
    public void publishDashboardCommands() {
        SmartDashboard.putData(
            TABLE + "RunAllChecks",
            dashboardButton(
                "RunAllChecks",
                this::getRunAllChecksCommand,
                subsystems.values().stream().filter(e -> e.systemCheckFactory != null).toList()
            )
        );
        SmartDashboard.putData(TABLE + "ClearAllFaults", getClearAllFaultsCommand());
        pollAll();
    }

    /**
     * Polls all hardware right now and publishes the results.
     */
    public void pollAll() {
        for (SubsystemEntry entry : subsystems.values()) {
            poll(entry);
        }
    }

    // ------------------------------------------------------------------
    // Internals
    // ------------------------------------------------------------------

    private SubsystemEntry entry(String subsystemName) {
        return subsystems.computeIfAbsent(subsystemName, SubsystemEntry::new);
    }

    private Command wrapSystemCheck(SubsystemEntry entry) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                entry.checkFaults.clear();
                entry.checkRan = false;
                publish(entry);
            }),
            entry.systemCheckFactory.get(),
            Commands.runOnce(() -> {
                entry.checkRan = true;
                poll(entry);
            })
        )
            .finallyDo(interrupted -> {
                if (interrupted) {
                    entry.checkFaults.add(SubsystemFault.warning("System check was interrupted before it finished"));
                    publish(entry);
                }
            })
            .onlyIf(DriverStation::isTest)
            .unless(DriverStation::isFMSAttached)
            .withName(entry.name + "Check");
    }

    /**
     * A dashboard button that works even while disabled: if the robot is
     * enabled in Test mode it starts a fresh copy of the check, otherwise it
     * adds a warning explaining how to run it (instead of silently doing nothing).
     */
    private Command dashboardButton(String name, Supplier<Command> check, List<SubsystemEntry> entries) {
        return Commands.runOnce(() -> {
            if (DriverStation.isFMSAttached()) return;
            if (DriverStation.isTestEnabled()) {
                CommandScheduler.getInstance().schedule(check.get());
                return;
            }
            for (SubsystemEntry entry : entries) {
                entry.checkFaults.add(
                    SubsystemFault.warning("System check not run - put the robot on blocks and enable it in Test mode")
                );
                publish(entry);
            }
        })
            .ignoringDisable(true)
            .withName(name);
    }

    private void pollWhenDisabled() {
        // Only poll while disabled so matches aren't slowed down. Skip in
        // simulation, where most device faults don't mean anything.
        if (DriverStation.isDisabled() && RobotBase.isReal()) {
            pollAll();
        }
    }

    private void poll(SubsystemEntry entry) {
        entry.hardwareFaults.clear();
        if (RobotBase.isReal()) {
            for (SelfChecking device : entry.hardware) {
                entry.hardwareFaults.addAll(device.checkForFaults());
            }
        }
        publish(entry);
    }

    private void publish(SubsystemEntry entry) {
        Set<SubsystemFault> faults = entry.allFaults();
        SystemStatus status = SystemStatus.of(faults);
        String table = TABLE + entry.name;

        SmartDashboard.putString(table + "/Status", status.name());
        SmartDashboard.putBoolean(table + "/SystemOK", status != SystemStatus.ERROR);
        SmartDashboard.putBoolean(table + "/CheckRan", entry.checkRan);
        SmartDashboard.putStringArray(
            table + "/Faults",
            faults.stream().map(f -> (f.isWarning() ? "WARNING: " : "ERROR: ") + f.description()).toArray(String[]::new)
        );
        SmartDashboard.putBoolean(TABLE + "AllOK", allOK());

        // Keep exactly one active Alert per current fault.
        entry.alerts
            .entrySet()
            .removeIf(e -> {
                if (faults.contains(e.getKey())) return false;
                e.getValue().set(false);
                e.getValue().close();
                return true;
            });
        for (SubsystemFault fault : faults) {
            entry.alerts.computeIfAbsent(fault, f -> {
                Alert alert = new Alert(
                    entry.name + ": " + f.description(),
                    f.isWarning() ? AlertType.kWarning : AlertType.kError
                );
                alert.set(true);
                return alert;
            });
        }
    }
}
