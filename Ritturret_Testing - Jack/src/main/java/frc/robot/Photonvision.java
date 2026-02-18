package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;

//HD_Pro_Webcam_C920
public class Photonvision {
    PhotonCamera camera1 = new PhotonCamera("FULL_HD_1080P_Webcam");
    ADIS16470_IMU gyro;
    private TalonFX motor = new TalonFX(5);
    public final int[] priorityOneTags  = {18, 21, 26}; //Tags That are Prioritsed over all others
    public final int[] priorityTwoTags  = {17, 19, 20, 25, 27}; //Tags That are Prioritsed second
    private double VISION_TURN_kP = -0.5;
    private double TEST_speed = 12/Math.PI;
    private double TESTING_taglook = 0;
    private int lastTargetYawPosOrNeg = 0;
    private boolean testthing = false;
    public PhotonPoseEstimator photonEstimator;
    private Transform3d RedGoalCenter;
    private Transform3d BlueGoalCenter;
    public String AllianceColor;
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
    List<PhotonTrackedTarget> targetList = camera1.getLatestResult().getTargets();
    AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    
    public void Init(String Color)
    {
        AllianceColor = Color;
        //gyro = new ADIS16470_IMU();
        RedGoalCenter = GetGoalCenter("Red");
        BlueGoalCenter = GetGoalCenter("Blue");
    }


    public void CameraTargetList()
    {
        targetList = camera1.getLatestResult().getTargets();
        for (var i = 0; i < targetList.size(); i++)
        {
            SmartDashboard.putNumber("List of Targets that can be seen (" + i + ")", targetList.get(i).getFiducialId());
            SmartDashboard.putNumber("Target (" + i + ")'s Skew", targetList.get(i).getSkew());
        }
    }

    private Transform3d GetGoalCenter(String Team)
    {
        ArrayList<Integer> GoalTags = new ArrayList<Integer>();
        double CenterX = 0.0;
        double CenterY = 0.0;
        double CenterZ = 1.12395;
        if (Team == "Red")
        {
            GoalTags.add(10);
            GoalTags.add(5);
            GoalTags.add(4);
            GoalTags.add(2);
        }
        else if (Team == "Blue")
        {
            GoalTags.add(26);
            GoalTags.add(18);
            GoalTags.add(20);
            GoalTags.add(21);
        }
        CenterX = getAverageDoubleList(new ArrayList<Double>(Arrays.asList(tagLayout.getTagPose(GoalTags.get(0)).get().getX(), tagLayout.getTagPose(GoalTags.get(2)).get().getX())));
        CenterY = getAverageDoubleList(new ArrayList<Double>(Arrays.asList(tagLayout.getTagPose(GoalTags.get(1)).get().getY(), tagLayout.getTagPose(GoalTags.get(3)).get().getY())));
        SmartDashboard.putNumber("CenterX_"+Team, CenterX);
        SmartDashboard.putNumber("CenterY_"+Team, CenterY);
        SmartDashboard.putNumber("CenterZ_"+Team, CenterZ);
        return new Transform3d(CenterX,CenterY,CenterZ, new Rotation3d());
    }

    private double CalculateAim(Transform3d RobotPos)
    {
        SmartDashboard.putString("Alliance Color", AllianceColor);
        var motorTicks = motor.getPosition().getValue();
        photonEstimator = new PhotonPoseEstimator(tagLayout, RobotPos);
        Pose2d RobotPose = new Pose2d(RobotPos.getX(),RobotPos.getY(), new Rotation2d(/*gyro.getAngle()*/ motorTicks));
        Pose2d TargetPose = new Pose2d(0.0,0.0, new Rotation2d()); 
        double yawReturn = 0.0;
        if (AllianceColor == "Blue")
        {
            TargetPose = new Pose2d(BlueGoalCenter.getX(),BlueGoalCenter.getY(), new Rotation2d(BlueGoalCenter.getRotation().getAngle())); 
        }
        else if(AllianceColor == "Red")
        {
            TargetPose = new Pose2d(RedGoalCenter.getX(),RedGoalCenter.getY(), new Rotation2d(RedGoalCenter.getRotation().getAngle())); 
        }
        yawReturn = PhotonUtils.getYawToPose(RobotPose, TargetPose).getRadians();
        SmartDashboard.putNumber("("+ AllianceColor +") Aim Yaw", yawReturn);
        return yawReturn;
    }
    public double GetTargetAim()
    {
        return CalculateAim(getRobotPosEstimate());
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
                double TargetMulti = 1;
                double TargetUpdate = targetYaw * TargetMulti;
                double speedMod = -1 * (TargetUpdate/100);
                double speedBoost = 1;
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

    //Random Math Average thingy, returns a double of the values of provided ArrayList
    public double getAverageDoubleList(ArrayList<Double> List)
    {
        double Average; //Final return value
        //The Average Value
        double AveragePlus = 0.0;
        //For Loop to get diffrent estimates
        for (var i=0; i < List.size(); i++)
        {
            AveragePlus += List.get(i);
        }
        //Save List Size as a Variable
        int Size = List.size();
        //Divide the Averages by the size to get the average
        Average = AveragePlus/Size;
        return Average;
    }
    public double CameraGetTargetYaw()
    {
        var testingthing = 0.0;
        Transform3d EsimatedPos = getRobotPosEstimate();
        Transform3d TagPosAverage = getTargetPosEstimate();
        SmartDashboard.putNumber("TESTING_taglook", TESTING_taglook);
        if (TESTING_taglook > 0)
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
        else if (TESTING_taglook == -1)
        {
            return getTargetPosEstimate().getX();
        }
        else if (TESTING_taglook == -2)
        {
            ArrayList<Double> Return = new ArrayList<Double>();
            for (var i =0; i < targetList.size(); i++)
            {
                Return.add(targetList.get(i).getYaw());
            }
            return getAverageDoubleList(Return);
        }
        else if (TESTING_taglook == 0)
        {
            var returnval = GetTargetAim();
            SmartDashboard.putNumber("CameraGetTargetYaw()", returnval);
            return returnval;
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

    public Pose3d PosEstimate(PhotonTrackedTarget target)
    {
        photonEstimator = new PhotonPoseEstimator(tagLayout, kRobotToCam);
        var result = camera1.getLatestResult();
        var Esitmate = photonEstimator.estimateCoprocMultiTagPose(result);
        if (Esitmate.isEmpty()) {
            Esitmate = photonEstimator.estimateLowestAmbiguityPose(result);
        }
        return Esitmate.get().estimatedPose;
    }

    private Transform3d getCamTagPos(PhotonTrackedTarget target)
    {
        int TagID = target.getFiducialId();
        Transform3d camToTarget = target.bestCameraToTarget;   

        Pose3d fieldToTag = tagLayout.getTagPose(TagID).get();

        fieldToTag.plus(camToTarget.inverse());

        return new Transform3d(fieldToTag.getX(),fieldToTag.getY(),fieldToTag.getZ(),fieldToTag.getRotation());
    }


    public Pose3d Transform3dtoPose3d(Transform3d input)
    {
        return new Pose3d(input.getX(),input.getY(),input.getZ(),input.getRotation());
    }
    public Transform3d Pose3dtoTransform3d(Pose3d input)
    {
        return new Transform3d(input.getX(),input.getY(),input.getZ(),input.getRotation());
    }
    public Transform3d getRobotPosEstimate()
    {
        Transform3d RobotPosEstimate; //Final return value
        // List of the Position Esitmates, Needed and value to start, so added a junk value
        ArrayList<Transform3d> RobotPosEstimates = new ArrayList<Transform3d>();
        //The Average Values
        double AverageX = 0.0;
        double AverageY = 0.0;
        double AverageZ = 0.0;
        double AverageRoll = 0.0;
        double AveragePitch = 0.0;
        double AverageYaw = 0.0;
        //For Loop to get diffrent estimates
        for (var i=0; i < targetList.size(); i++)
        {
            Transform3d Temp_robotPos = Pose3dtoTransform3d(PosEstimate(targetList.get(i))); //Gets the Robot position according to that tag
            RobotPosEstimates.add(Temp_robotPos);
            AverageX += Temp_robotPos.getX(); // Add the diffrent values to the Average Variables
            AverageY += Temp_robotPos.getY();
            AverageZ += Temp_robotPos.getZ();
            AverageRoll += Temp_robotPos.getRotation().getX();
            AveragePitch += Temp_robotPos.getRotation().getY();
            AverageYaw += Temp_robotPos.getRotation().getZ();
        }
        //Save List Size as a Variable
        int Size = RobotPosEstimates.size();
        //Divide the Averages by the size to get the average
        AverageX = AverageX/Size;
        AverageY = AverageY/Size;
        AverageZ = AverageZ/Size;
        AverageRoll = AverageRoll/Size;
        AveragePitch = AveragePitch/Size;
        AverageYaw = AverageYaw/Size;
        //Smart Dashboard
        SmartDashboard.putNumber("Estimated X Pos", AverageX);
        SmartDashboard.putNumber("Estimated Y Pos", AverageY);
        SmartDashboard.putNumber("Estimated Z Pos", AverageZ);
        SmartDashboard.putNumber("Estimated Roll", AverageRoll);
        SmartDashboard.putNumber("Estimated Pitch", AveragePitch);
        SmartDashboard.putNumber("Estimated Yaw", AverageYaw);
        //Set Var and return the final value
        RobotPosEstimate = new Transform3d(AverageX, AverageY, AverageZ, new Rotation3d(AverageRoll, AveragePitch, AverageYaw));
        return RobotPosEstimate;
    }
    public Transform3d getTargetPosEstimate()
    {
        Transform3d RobotPosEstimate; //Final return value
        // List of the Position Esitmates, Needed and value to start, so added a junk value
        ArrayList<Transform3d> RobotPosEstimates = new ArrayList<Transform3d>();
        //The Average Values
        double AverageX = 0.0;
        double AverageY = 0.0;
        double AverageZ = 0.0;
        double AverageRoll = 0.0;
        double AveragePitch = 0.0;
        double AverageYaw = 0.0;
        //For Loop to get diffrent estimates
        for (var i=0; i < targetList.size(); i++)
        {
            Transform3d Temp_robotPos = targetList.get(i).bestCameraToTarget; 
            RobotPosEstimates.add(Temp_robotPos);
            AverageX += Temp_robotPos.getX(); // Add the diffrent values to the Average Variables
            AverageY += Temp_robotPos.getY();
            AverageZ += Temp_robotPos.getZ();
            AverageRoll += Temp_robotPos.getRotation().getX();
            AveragePitch += Temp_robotPos.getRotation().getY();
            AverageYaw += Temp_robotPos.getRotation().getZ();
        }
        //Save List Size as a Variable
        int Size = RobotPosEstimates.size();
        //Divide the Averages by the size to get the average
        AverageX = AverageX/Size;
        AverageY = AverageY/Size;
        AverageZ = AverageZ/Size;
        AverageRoll = AverageRoll/Size;
        AveragePitch = AveragePitch/Size;
        AverageYaw = AverageYaw/Size;
        //Smart Dashboard
        SmartDashboard.putNumber("Estimated X Pos (Tag)", AverageX);
        SmartDashboard.putNumber("Estimated Y Pos (Tag)", AverageY);
        SmartDashboard.putNumber("Estimated Z Pos (Tag)", AverageZ);
        SmartDashboard.putNumber("Estimated Roll (Tag)", AverageRoll);
        SmartDashboard.putNumber("Estimated Pitch (Tag)", AveragePitch);
        SmartDashboard.putNumber("Estimated Yaw (Tag)", AverageYaw);
        //Set Var and return the final value
        RobotPosEstimate = new Transform3d(AverageX, AverageY, AverageZ, new Rotation3d(AverageRoll, AveragePitch, AverageYaw));
        return RobotPosEstimate;
    }
    public void CameraUpdate()
    {
        /* 
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
        */

    }
}
