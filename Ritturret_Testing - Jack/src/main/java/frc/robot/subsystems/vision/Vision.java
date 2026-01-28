package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public class Vision {
    PhotonCamera camera1 = new PhotonCamera("Camera1");
    AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    Rotation2d yaw;
    public void update() {
        for (var result : camera1.getAllUnreadResults()){
         yaw = Rotation2d.fromDegrees(result.targets.get(0).getYaw());

        }
       if (yaw != null){

       }
       
    }
}
