package org.team1126.robot.util.nav;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

public interface Navigator {
    public Command driveWaypoint(WaypointHeading direction, BooleanSupplier left, int index);
    public Command heading(WaypointHeading heading);
}
