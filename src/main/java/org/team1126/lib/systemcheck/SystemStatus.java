package org.team1126.lib.systemcheck;

import java.util.Collection;

/**
 * The overall health of a subsystem, determined by its worst fault.
 */
public enum SystemStatus {
    /** No faults. */
    OK,
    /** Only warnings. */
    WARNING,
    /** At least one error. */
    ERROR;

    /**
     * Returns the status for a set of faults: ERROR if any fault is an error,
     * WARNING if there are only warnings, otherwise OK.
     * @param faults The faults to evaluate.
     */
    public static SystemStatus of(Collection<SubsystemFault> faults) {
        SystemStatus worst = OK;
        for (SubsystemFault fault : faults) {
            if (!fault.isWarning()) return ERROR;
            worst = WARNING;
        }
        return worst;
    }
}
