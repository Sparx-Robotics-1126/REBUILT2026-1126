package org.team1126.robot.util.autos;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Interface to wrap autos routines. This provides a nice wrapper to include the routines
 * in the autos chooser and then to simply execute them.
 *
 * <em>A note about autonomous routines:</em>
 *
 * All autos are assumed to be starting <i>"south"</i> of any major repulsive obstacle, and
 * if they are potentially directional, the routine is designed, by default, to go to the
 * <i>right</i>. This can be modified by passing the routine the AutosFlip enum, which will
 * either make the autos be indicated that there is no flip possible (which it would already
 * be configured for, but is being included for completeness sake), or it would allow the
 * inverse of the auto to be called.
 */
public interface AutosRoutine {
    /**
     * The action of the routine, creating the command to be executed as when the
     * autos are executed.
     *
     * @param flip whether or not to flip to the left or keep it right.
     * @return the Command to invoke as part of the autos.
     */
    public Command action(Supplier<AutosFlip> flip);

    public Command action(Supplier<AutosFlip> flip, BooleanSupplier blue);

    /**
     * This will create the action command for the routine with the ability to determine
     * a starting point that is within the set of the AutosStart enum. This is to be used
     * with autos that will shoot first and will be starting within one of the pre-determined
     * shooting locations.
     *
     * @param startAt the starting point for the autos.
     * @param flip whether or not to flip to the left or keep it right.
     * @return the COmmand to invoke in autos.
     */
    public Command action(Supplier<AutosStart> startAt, Supplier<AutosFlip> flip);

    public Command action(Supplier<AutosStart> startAt, Supplier<AutosFlip> flip, BooleanSupplier blue);

    /**
     * The value that this routine will name the action.
     *
     * @return the action name for the routine.
     */
    public String getCommandName();

    /**
     * The display name that will be shown in the autos chooser.
     *
     * @return the name of the command to display in the autos chooser.
     */
    public String getDisplayName();
    public String getDisplayName(AutosFlip flip);
    public String getDisplayName(AutosFlip flip, boolean succinct);
    public String getDisplayName(AutosStart startAt);
    public String getDisplayName(AutosStart startAt, boolean succinct);
    public String getDisplayName(AutosStart startAt, AutosFlip flip);
    public String getDisplayName(AutosStart startAt, AutosFlip flip, boolean succinct);
}
