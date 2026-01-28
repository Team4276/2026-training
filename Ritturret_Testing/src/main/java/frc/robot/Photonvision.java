package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose3d;

//HD_Pro_Webcam_C920
public class Photonvision {
    PhotonCamera camera1 = new PhotonCamera("Camera1");

    public final int[] priorityOneTags  = {18, 21, 26}; //Tags That are Prioritsed over all others
    public final int[] priorityTwoTags  = {17, 19, 20, 24, 27}; //Tags That are Prioritsed second

    AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);


    public double CameraAim(boolean PressedA)
    {
        // Read in relevant data from the Camera
        boolean targetVisible = false;
        double targetYaw = 0.0;
        var results = camera1.getAllUnreadResults();
        SmartDashboard.putBoolean("!results.isEmpty()", !results.isEmpty());
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            SmartDashboard.putBoolean("result.hasTargets()", result.hasTargets());
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (CheckIDs(target)) {
                        // Found Tag, record its information
                        targetYaw = target.getYaw();
                        targetVisible = true;
                    }
                }
            }
        }

        if (PressedA && targetVisible) {
            // Driver wants auto-alignment to tag 7
            // And, tag 7 is in sight, so we can turn toward it.
            // Override the driver's turn command with an automatic one that turns toward the tag.
            double VISION_TURN_kP = 1.0;
            return (double) -1.0 * targetYaw * VISION_TURN_kP;
        }
        else
        {
            return 0.0;
        }
    }

    private boolean CheckIDs(PhotonTrackedTarget target)
    {
        boolean returnval = false;
        for (var i = 0; i > priorityOneTags.length; i++)
        {
            if (target.getFiducialId() == priorityOneTags[i])
            {
                returnval = true;
                return returnval;
            }
        }
        for (var i = 0; i > priorityTwoTags.length; i++)
        {
            if (target.getFiducialId() == priorityTwoTags[i])
            {
                returnval = true;
                return returnval;
            }
        }
        SmartDashboard.putBoolean("April Tag ID matches List", returnval);
        return returnval;
        
    }
    public void CameraUpdate()
    {

        boolean canSeeTag = camera1.getLatestResult().hasTargets();

        SmartDashboard.putBoolean("Can I see an April Tag?", canSeeTag);
        if (!canSeeTag)
        {
            return;
        }
        var result = camera1.getAllUnreadResults().get(0);
        boolean ItWorked = false;

        Transform3d camToTarget = result.targets.get(0).bestCameraToTarget;   

        Pose3d fieldToTag = tagLayout.getTagPose(1).get();

        SmartDashboard.putNumber("Get X", camToTarget.getX());

        // camToTarget.getX();

        ///fieldToTag.plus(camToTarget.inverse());

    }
}
