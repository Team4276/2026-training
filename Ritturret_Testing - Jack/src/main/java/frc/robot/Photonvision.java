package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.List;
import java.util.ArrayList;

//HD_Pro_Webcam_C920
public class Photonvision {
    PhotonCamera camera1 = new PhotonCamera("FULL_HD_1080P_Webcam");

    public final int[] priorityOneTags  = {18, 21, 26}; //Tags That are Prioritsed over all others
    public final int[] priorityTwoTags  = {17, 19, 20, 25, 27}; //Tags That are Prioritsed second
    private double VISION_TURN_kP = -0.5;
    private double TEST_speed = 12/Math.PI;
    private int TESTING_taglook = 18;
    private int lastTargetYawPosOrNeg = 0;
    List<PhotonTrackedTarget> targetList = camera1.getLatestResult().getTargets();
    AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public void CameraTargetList()
    {
        targetList = camera1.getLatestResult().getTargets();
        for (var i = 0; i < targetList.size(); i++)
        {
            SmartDashboard.putNumber("List of Targets that can be seen (" + i + ")", targetList.get(i).getFiducialId());
        }
    }
    public double CameraAimB(boolean PressedB,boolean PressedX)
    {
        boolean targetVisible = !targetList.isEmpty();
        double targetYaw = 0.0;

        /*var results = camera1.getAllUnreadResults();
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
        }*/
        targetYaw = CameraGetTargetYaw();
        if (PressedB && targetVisible) {
            if ((Math.abs(targetYaw) < 0.2) || (Math.abs(targetYaw) > 20.0))
            {
                VISION_TURN_kP = VISION_TURN_kP * -1;

            }
            // Driver wants auto-alignment to tag 
            // And, tag is in sight, so we can turn toward it.
            // Override the driver's turn command with an automatic one that turns toward the tag.
            if (PressedX)
            {
                double speedMod = -1 * (targetYaw/100);
                double speedBoost = SmartDashboard.getNumber("SpeedBoost", 1);
                double speed = TEST_speed * speedMod * speedBoost;
                if (targetYaw > 0)
                {
                    return speed;
                }
                else if (targetYaw < 0)
                {
                    return speed;
                }
            }
            return (double) -1.0 * targetYaw * VISION_TURN_kP;
        }
        else
        {
            return 0.0;
        }
    }


    public double CameraGetTargetYaw()
    {
        var testingthing = 0.0;
        if (TESTING_taglook != -1)
        {
            SmartDashboard.putNumber("targetList.size()", (double)targetList.size());
            for (var i = 0; i < targetList.size(); i++)
            {
                SmartDashboard.putNumber("TESTING_taglook", TESTING_taglook);
                SmartDashboard.putNumber("TagID ("+i+")", targetList.get(i).getFiducialId());
                if (targetList.get(i).getFiducialId() == TESTING_taglook)
                {
                    testingthing = targetList.get(i).getYaw();
                    SmartDashboard.putNumber("CameraGetTargetYaw()", testingthing);
                    if (testingthing > 0)
                    {
                        lastTargetYawPosOrNeg = 1;
                    }
                    else if (testingthing < 0)
                    {
                        lastTargetYawPosOrNeg = -1;
                    }
                    SmartDashboard.putNumber("Last Target Yaw Pos or Neg?", lastTargetYawPosOrNeg);
                    return testingthing;
                }
            }
            
        }
        SmartDashboard.putNumber("CameraGetTargetYaw()", testingthing);
        return 0.0;
        
    }


    private boolean CheckIDs(PhotonTrackedTarget target)
    {
        boolean returnval = false;
        int TagID = target.getFiducialId();
        for (var i = 0; i < priorityOneTags.length; i++)
        {
            if (TagID == priorityOneTags[i])
            {
                returnval = true;
                return returnval;
            }
        }
        for (var i = 0; i < priorityTwoTags.length; i++)
        {
            if (TagID == priorityTwoTags[i])
            {
                returnval = true;
                return returnval;
            }
        }
        SmartDashboard.putBoolean("April Tag ID ("+ target.getFiducialId() +") matches List", returnval);
        return returnval;
        
    }

    public void CheckTagID(PhotonTrackedTarget target)
    {
        
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
