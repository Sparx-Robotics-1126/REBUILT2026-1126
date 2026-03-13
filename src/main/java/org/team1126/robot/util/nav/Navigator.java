package org.team1126.robot.util.nav;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.team1126.robot.util.autos.AutosFlip;

public interface Navigator {
    public Command driveWaypoint(Supplier<AutosFlip> flip, int index);
    public Command driveWaypoint(Supplier<WaypointHeading> direction, BooleanSupplier left, int index);
    public Command driveWaypoint(Supplier<AutosFlip> flip, int index, BooleanSupplier blue);
    public Command driveWaypoint(
        Supplier<WaypointHeading> direction,
        BooleanSupplier left,
        int index,
        BooleanSupplier blue
    );
    public Command heading(Supplier<WaypointHeading> heading);
}
