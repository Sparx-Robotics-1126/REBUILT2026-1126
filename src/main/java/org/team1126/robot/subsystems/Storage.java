package org.team1126.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.photonvision.PhotonCamera;
import org.team1126.lib.util.command.GRRSubsystem;
import org.team1126.robot.Constants;

@Logged
public final class Storage extends GRRSubsystem {

    private PhotonCamera camera;
    private int fuelCount;
    private IntegerPublisher fuelCountPublisher;

    public Storage() {
        camera = new PhotonCamera(Constants.OBJ_DETECTION_CAMERA_CONFIG.name());
        fuelCount = 0;

        NetworkTableInstance nti = NetworkTableInstance.getDefault();
        NetworkTable table = nti.getTable("vision");
        fuelCountPublisher = table.getIntegerTopic("fuelCount").publish();
    }

    private void updateFuelCount() {
        try {
            var results = camera.getAllUnreadResults();
            fuelCount = results.get(0).getTargets().size();
        } catch (IndexOutOfBoundsException iOOBE) {}
    }

    @Override
    public void periodic() {
        updateFuelCount();
        fuelCountPublisher.set(fuelCount);
    }
}
