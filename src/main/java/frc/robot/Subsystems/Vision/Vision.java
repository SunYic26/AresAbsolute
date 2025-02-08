package frc.robot.Subsystems.Vision;    

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

// import org.littletonrobotics.junction.Logger;
import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.MultiTagOutput;
import frc.lib.VisionOutput;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants.VisionLimits;
import frc.robot.LimelightHelpers;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;

public class Vision extends SubsystemBase {
    private static Vision instance;
    private static PhotonCamera centerCamera;

    List<MultiTagOutput> multiTagResults = new ArrayList<>();

    private static List<PhotonPipelineResult> cameraResult = new ArrayList<>();

    private List<Double> lastProcessedTimestamp = new ArrayList<>();

    CommandSwerveDrivetrain s_Swerve;
    LimelightSubsystem s_Lime;
    RobotState robotState;
    
    public double floorDistance;

    private Transform3d cameraToRobotTransform = new Transform3d(
        //center cam
        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(-5.67162), Units.inchesToMeters(-10.172538)),
        new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(40),Units.degreesToRadians(0)));

    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraToRobotTransform);
    
    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }
    
    public Vision() {
        s_Swerve = CommandSwerveDrivetrain.getInstance();
        robotState = RobotState.getInstance();


        centerCamera = new PhotonCamera(Constants.VisionConstants.cameraName);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);
        updateAprilTagResults();
    }

    public void updateAprilTagResults() {
        cameraResult.clear();
        cameraResult.add(centerCamera.getLatestResult());

        // Zero yaw is robot facing red alliance wall - our code should be doing this.
    }

    public List<List<PhotonTrackedTarget>> getAllTargets(){
        List<List<PhotonTrackedTarget>> targets = new ArrayList<>();

        for (PhotonPipelineResult result : cameraResult) {
            if (!targets.isEmpty())
                targets.add(result.getTargets());
        }

        return targets;
    }

    public List<PhotonPipelineResult> getValidTargets(List<PhotonPipelineResult> camera) {
        List<PhotonPipelineResult> results = new ArrayList<>();

        for (PhotonPipelineResult photonPipelineResult : camera) {
            PhotonTrackedTarget target = photonPipelineResult.getBestTarget();
            if(photonPipelineResult.hasTargets()
            && target.getFiducialId() >= 1
            && target.getFiducialId() <= Constants.VisionConstants.aprilTagMax
            && target.getPoseAmbiguity() < 0.2 && target.getPoseAmbiguity() > -1) {
            results.add(photonPipelineResult);
            } 
        }

        return results;
    }

    private List<MultiTagOutput> getMultiTags() {
        List<MultiTagOutput> multiTagResults = new ArrayList<>();

        for (PhotonPipelineResult photonPipelineResult : cameraResult) {
            if(photonPipelineResult.getMultiTagResult().isPresent() && multitagChecks(photonPipelineResult.getMultiTagResult().get())) {
                multiTagResults.add(new MultiTagOutput(photonPipelineResult.getMultiTagResult().get(), photonPipelineResult.getTimestampSeconds(), photonPipelineResult.getBestTarget()));
            }
        }

        return multiTagResults;
    }

    private Boolean multitagChecks(MultiTargetPNPResult multiTagResult) {

        if(multiTagResult.estimatedPose.bestReprojErr > VisionLimits.k_reprojectionLimit) {
            SmartDashboard.putString("Multitag updates", "high error");
            // Logger.recordOutput("Vision/MultiTag updates", "high error");
            return false;
        }
        if(multiTagResult.fiducialIDsUsed.size() < 2 || multiTagResult.fiducialIDsUsed.isEmpty()) {
            SmartDashboard.putString("Multitag updates", "insufficient ids");
            // Logger.recordOutput("Vision/MultiTag updates", "insufficient ids");
            return false;
        } 
        if(multiTagResult.estimatedPose.best.getTranslation().getNorm() < VisionLimits.k_normThreshold) {
            SmartDashboard.putString("Multitag updates", "norm check failed");
            // Logger.recordOutput("Vision/MultiTag updates", "norm check failed");
            return false;
        } 
        if(multiTagResult.estimatedPose.ambiguity > VisionLimits.k_ambiguityLimit) {
            SmartDashboard.putString("Multitag updates", "high ambiguity");
            // Logger.recordOutput("Vision/MultiTag updates", "high ambiguity");
            return false;
        }

        //in the future we would have a set of tags we would only want to mega tag
        // for (var fiducialID : multiTagResult.fiducialIDsUsed) {
        //     if (fiducialID =! idk) {
        //     }
        // }

        return true;
    }

    private void checkTimestamps() {
        for (PhotonPipelineResult photonPipelineResult : cameraResult) {
            if (lastProcessedTimestamp.contains(photonPipelineResult.getTimestampSeconds())) {
                cameraResult.remove(photonPipelineResult);
            }
        }

        if(!lastProcessedTimestamp.isEmpty())
            lastProcessedTimestamp.clear();

        for (PhotonPipelineResult photonPipelineResult : cameraResult) {
            lastProcessedTimestamp.add(photonPipelineResult.getTimestampSeconds());
        }
    }

    /**
     * calculates field-relative robot pose from vision reading, feed to pose estimator (Kalman filter)
     */
    private void updateVision() throws Exception {

        //ensure this works
        if(Math.abs(robotState.robotAngularVelocityMagnitude()[0]) > VisionLimits.k_rotationLimitDPS) {
            SmartDashboard.putString("Vision accepter", "Vision failed: High rotation");
            return;
        }
        
        checkTimestamps();

        //limelight update
        if(s_Lime.hasTarget()) {
            LimelightHelpers.SetRobotOrientation(Constants.LimelightConstants.cameraName, robotState.robotYaw().getDegrees(), 0,0 ,0,0,0);
            VisionOutput pose = new VisionOutput(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LimelightConstants.cameraName));
            robotState.visionUpdate(pose);
        }
        
        multiTagResults = getMultiTags();

        if(!multiTagResults.isEmpty()) { //Use multitag if available
            for (MultiTagOutput multiTagResult : multiTagResults) {

                Pose3d tagPose = aprilTagFieldLayout.getTagPose(multiTagResult.getBestTarget().getFiducialId()).get();

                Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(multiTagResult.estimatedPose.best, tagPose, cameraToRobotTransform);

                VisionOutput newPose = new VisionOutput(robotPose, multiTagResult.getTimestamp(),  multiTagResult.getBestTarget(), PoseStrategy.CLOSEST_TO_LAST_POSE);
                
                System.out.println(newPose.toString());

                robotState.visionUpdate(newPose); 
            }
        } else { // if no multitags, use single tag
            List<PhotonPipelineResult> results = getValidTargets(cameraResult);

            if(!results.isEmpty()) {
                for (PhotonPipelineResult photonPipelineResult : results) {
                    VisionOutput newPose = new VisionOutput(photonPoseEstimator.update(photonPipelineResult).get());
                    System.out.println(newPose.estimatedPose.toString());
                    robotState.visionUpdate(newPose); 
                } 
            } else SmartDashboard.putString("Vision accepter", "Vision failed: No valid targets");
        }
    }

    @Override
    public void periodic() {
        updateAprilTagResults();
            try {
                updateVision();
            } catch (Exception e){}
    }
}