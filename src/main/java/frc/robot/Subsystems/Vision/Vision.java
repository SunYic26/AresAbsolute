package frc.robot.Subsystems.Vision;    

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.VisionOutput;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants.VisionLimits;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.CommandSwerveDrivetrain.Drivetrain;

import java.io.ObjectInputStream.GetField;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.UnitBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.littletonrobotics.junction.Logger;
import frc.robot.Subsystems.Vision.Vision;

public class Vision extends SubsystemBase {
    private static Vision instance;
    private static PhotonCamera centerCamera;

    // private static PhotonCamera backRightCamera;
    private static PhotonPipelineResult cameraResult;

    private double lastProcessedTimestamp = -1;

    Drivetrain s_Swerve;
    RobotState robotState;
    
    public double floorDistance;

    private Transform3d cameraToRobotTransform = new Transform3d(
        //center cam
        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(-5.67162), Units.inchesToMeters(-10.172538)),
        new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(40),Units.degreesToRadians(0)));

    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, centerCamera, cameraToRobotTransform);
    
    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }
    
    public Vision() {
        s_Swerve = Drivetrain.getInstance();
        robotState = RobotState.getInstance();

        centerCamera = new PhotonCamera(Constants.VisionConstants.cameraName);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);
        updateAprilTagResults();
    }

    public void updateAprilTagResults() {
        cameraResult = centerCamera.getLatestResult();
    }

    public PhotonPipelineResult getLatestAprilTagResult(){
        updateAprilTagResults();
        return cameraResult;
    }

    public List<PhotonTrackedTarget> getTargets(){
        return cameraResult.getTargets();
    }

    public Boolean hasValidTarget(PhotonPipelineResult camera) {
        if(camera.hasTargets()) {
            PhotonTrackedTarget target = camera.getBestTarget();

            return
            target.getFiducialId() >= 1 &&
            target.getFiducialId() <= Constants.VisionConstants.aprilTagMax &&
            target.getPoseAmbiguity() < 0.2 && target.getPoseAmbiguity() > -1;
        } else {return false;}
    }

    private Boolean shouldUseMultiTag() {
        MultiTargetPNPResult multiTagResult = cameraResult.getMultiTagResult();

        if(multiTagResult.estimatedPose.bestReprojErr > VisionLimits.k_reprojectionLimit) {
            SmartDashboard.putString("Multitag updates", "high error");
            return false;
        }
        if(multiTagResult.fiducialIDsUsed.size() < 2 || multiTagResult.fiducialIDsUsed.isEmpty()) {
            SmartDashboard.putString("Multitag updates", "insufficient ids");
            return false;
        } 
        if(multiTagResult.estimatedPose.best.getTranslation().getNorm() < VisionLimits.k_normThreshold) {
            SmartDashboard.putString("Multitag updates", "norm check failed");
            return false;
        } 
        if(multiTagResult.estimatedPose.ambiguity > VisionLimits.k_ambiguityLimit) {
            SmartDashboard.putString("Multitag updates", "high ambiguity");
            return false;
        }

        //in the future we would have a set of tags we would only want to mega tag

        // for (var fiducialID : multiTagResult.fiducialIDsUsed) {
        //     if (fiducialID =! idk) {
        //     }
        // }

        return true;
    }


    /**
     * calculates field-relative robot pose from vision reading, feed to pose estimator (Kalman filter)
     */
    public void updateVision() throws Exception{

        if(cameraResult.getTimestampSeconds() == lastProcessedTimestamp) {
            SmartDashboard.putString("Vision accepter", "Vision failed: old");
            return;
        }
        
        if(Math.abs(robotState.robotAngularVelocityMagnitude()[0]) > VisionLimits.k_rotationLimitDPS) {
            SmartDashboard.putString("Vision accepter", "Vision failed: High rotation");
            return;
        } 

        if(!cameraResult.getMultiTagResult().estimatedPose.isPresent) {
            if(hasValidTarget(cameraResult)) { //using fallback tag
                VisionOutput newPose = new VisionOutput(photonPoseEstimator.update().get());
                robotState.visionUpdate(newPose); 
                return;
            }
            
        } else if (shouldUseMultiTag()) { //using multitag

            VisionOutput newPose = new VisionOutput(photonPoseEstimator.update().get());
            robotState.visionUpdate(newPose); 
            return;

        } else if (hasValidTarget(cameraResult)){ // manually making the pose

                Pose3d targetPose = aprilTagFieldLayout.getTagPose(cameraResult.getBestTarget().getFiducialId()).orElse(null);
                Pose3d newPose = PhotonUtils.estimateFieldToRobotAprilTag(
                cameraResult.getBestTarget().getBestCameraToTarget(), targetPose, cameraToRobotTransform);
                robotState.visionUpdate(new VisionOutput(newPose, cameraResult.getTimestampSeconds(),
                cameraResult.getBestTarget(), PoseStrategy.CLOSEST_TO_LAST_POSE));

        } 


    }
        // else { SmartDashboard.putString("Vision accepter", "Vision failed: no targets");} 
        //less nesting! 

        // lastProcessedTimestamp = cameraResult.getTimestampSeconds();

    @Override
    public void periodic() {
        updateAprilTagResults();
        if(!cameraResult.targets.isEmpty()) {
            try {
                updateVision();
            } catch (Exception e){}
        }
    }
}