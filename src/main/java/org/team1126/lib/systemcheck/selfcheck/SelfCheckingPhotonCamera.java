package org.team1126.lib.systemcheck.selfcheck;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.team1126.lib.systemcheck.SubsystemFault;

/**
 * Self-check for a PhotonVision camera. Only reads status values from
 * NetworkTables; it never reads pipeline results, so it can't steal frames
 * from the pose estimator.
 *
 * <p>Checks that the camera is connected (PhotonVision heartbeat), that it
 * has a calibration for its current resolution (needed for AprilTag pose
 * estimates), and that it isn't left in driver mode (no processing).
 */
public final class SelfCheckingPhotonCamera implements SelfChecking {

    private final PhotonCamera camera;
    private final String label;
    private final Debouncer disconnectedDebounce = new Debouncer(1.5, DebounceType.kRising);

    /**
     * @param camera The PhotonCamera object the robot already uses. Pass the
     *               existing object rather than creating a second one with the same name.
     */
    public SelfCheckingPhotonCamera(PhotonCamera camera) {
        this.camera = camera;
        this.label = "camera " + camera.getName();
    }

    @Override
    public List<SubsystemFault> checkForFaults() {
        List<SubsystemFault> faults = new ArrayList<>();

        if (disconnectedDebounce.calculate(!camera.isConnected())) {
            faults.add(
                SubsystemFault.error(
                    "[" + label + "] not connected - check coprocessor power and ethernet, and that the camera name matches the PhotonVision UI"
                )
            );
            return faults;
        }

        if (camera.getCameraMatrix().isEmpty()) {
            faults.add(
                SubsystemFault.error(
                    "[" + label + "] has no calibration for its current resolution - AprilTag poses won't work. Calibrate it in PhotonVision"
                )
            );
        }

        if (camera.getDriverMode()) {
            faults.add(SubsystemFault.warning("[" + label + "] is in driver mode - it isn't processing AprilTags"));
        }

        return faults;
    }
}
