package org.team1126.robot.util.nav;

import edu.wpi.first.wpilibj2.command.Command;

public interface Navigator {
    public Command driveWaypoint(WaypointHeading direction, int index);
    public Command heading(WaypointHeading heading);
}
