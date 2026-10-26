package org.team1126.lib.systemcheck;

/**
 * A single problem found with a subsystem, either by a device self-check
 * (disconnected motor, brownout, overheating, ...) or by a subsystem's
 * system check command (flywheel didn't reach speed, module angle off, ...).
 *
 * <p>Two faults with the same description and severity are considered equal,
 * so the same problem is only reported once.
 *
 * @param description A human readable description, shown on the dashboard.
 * @param isWarning {@code true} for a warning (robot can still run, but look at it),
 *                  {@code false} for an error (something is broken).
 */
public record SubsystemFault(String description, boolean isWarning) {
    /**
     * Creates an error fault (something is broken).
     * @param description A human readable description of the fault.
     */
    public static SubsystemFault error(String description) {
        return new SubsystemFault(description, false);
    }

    /**
     * Creates a warning fault (robot can still run, but someone should look at it).
     * @param description A human readable description of the fault.
     */
    public static SubsystemFault warning(String description) {
        return new SubsystemFault(description, true);
    }
}
