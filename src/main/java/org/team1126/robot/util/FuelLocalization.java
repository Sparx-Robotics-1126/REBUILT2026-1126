package org.team1126.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class FuelLocalization {

  private FuelLocalization() {}

  /**
   * Estimates the field position of a target on the carpet (z=0) from camera yaw/pitch.
   *
   * Assumptions:
   *  - fuel is on the carpet (z = 0)
   *  - robot pose is accurate enough
   *  - robotToCamera is measured correctly
   *
   * @return field-relative Pose2d at the hit point, or null if geometry is invalid.
   */
  public static Pose2d estimateOnCarpet(
      Pose2d fieldToRobot,
      Transform3d robotToCamera,
      double yawDeg,
      double pitchDeg
  ) {
    // Angles from PhotonVision are in degrees.
    double yaw = Units.degreesToRadians(yawDeg);
    double pitch = Units.degreesToRadians(pitchDeg);

    // Build a unit direction vector in CAMERA coordinates.
    // Convention here: +X forward, +Y left, +Z up.
    // If your result is mirrored, flip yaw or swap sin/cos on Y.
    double dx = Math.cos(pitch) * Math.cos(yaw);
    double dy = Math.cos(pitch) * Math.sin(yaw);
    double dz = Math.sin(pitch);
    Translation3d rayCam = new Translation3d(dx, dy, dz);

    // Field->Camera pose:
    Pose3d fieldToCamera = new Pose3d(fieldToRobot).transformBy(robotToCamera);

    // Rotate ray into FIELD coordinates using camera rotation:
    Rotation3d camRot = fieldToCamera.getRotation();
    Translation3d rayField = rayCam.rotateBy(camRot);

    // Ray-plane intersection with z=0 (carpet):
    double camZ = fieldToCamera.getZ();
    double dirZ = rayField.getZ();
    if (Math.abs(dirZ) < 1e-6) return null; // nearly parallel to carpet

    double t = (0.0 - camZ) / dirZ;
    if (t <= 0.0) return null; // intersection behind camera

    Translation3d hit = fieldToCamera.getTranslation().plus(rayField.times(t));
    return new Pose2d(new Translation2d(hit.getX(), hit.getY()), fieldToRobot.getRotation());
  }
}
