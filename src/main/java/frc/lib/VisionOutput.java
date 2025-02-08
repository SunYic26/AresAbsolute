package frc.lib;

import java.util.Collections;
import java.util.List;

import frc.robot.LimelightHelpers;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.Interpolating.Geometry.ITranslation2d;

import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
/**
 * VisionOutput
 */
public class VisionOutput {

    /** The estimated pose */
    public final Pose3d estimatedPose;

    /** The estimated time the frame used to derive the robot pose was taken */
    public final double timestampSeconds;

    /** A list of the targets used to compute this pose */
    public final List<PhotonTrackedTarget> targetsUsed;

    public final double standardDev;

    public VisionOutput(Pose3d estimatedPose, double timestampSeconds, List<PhotonTrackedTarget> targetsUsed)  {
        this.estimatedPose = estimatedPose;
        this.timestampSeconds = timestampSeconds;
        this.targetsUsed = targetsUsed;
        this.standardDev = getStandardDeviation(targetsUsed);
    }

    public VisionOutput(Pose3d estimatedPose, double timestampSeconds, double standardDev)  {
        this.estimatedPose = estimatedPose;
        this.timestampSeconds = timestampSeconds;
        this.targetsUsed = null;
        this.standardDev = getStandardDeviation(targetsUsed);
    }

    public VisionOutput(Pose3d estimatedPose, double timestampSeconds, PhotonTrackedTarget targetsUsed)  {
        this.estimatedPose = estimatedPose;
        this.timestampSeconds = timestampSeconds;
        this.targetsUsed = Collections.singletonList(targetsUsed);
        this.standardDev = getStandardDeviation(this.targetsUsed);
    }

    public VisionOutput(EstimatedRobotPose pose)  {
        this(pose.estimatedPose, pose.timestampSeconds, pose.targetsUsed);
    }
    
    public VisionOutput(PoseEstimate poseEstimate){
        this(new Pose3d(poseEstimate.pose), poseEstimate.timestampSeconds, getStandardDeviation(poseEstimate));
    }




    private static double getStandardDeviation(PoseEstimate limelighEstimate) { 
        int count = 0;

        double meanArea = 0.0;
        double meanAmbiguity = 0.0;
        double sumSquareDiffArea = 0.0;
        double sumSquareDiffAmbiguity = 0.0;

        for (RawFiducial fiducials : limelighEstimate.rawFiducials) {
            count++;

            double deltaArea = fiducials.ta - meanArea;
            meanArea += deltaArea / count;
            sumSquareDiffArea += deltaArea * (fiducials.ta - meanArea);

            double deltaAmbiguity = fiducials.ambiguity - meanAmbiguity;
            meanAmbiguity += deltaAmbiguity / count;
            sumSquareDiffAmbiguity += deltaAmbiguity * (fiducials.ambiguity - meanAmbiguity);
        }

        //if we only have one target return default stdev
        if(count <= 1) {
            return 1e-4;
        } 

        double varianceArea = sumSquareDiffArea / (count - 1);
        double varianceAmbiguity = sumSquareDiffAmbiguity / (count - 1);

        // Return average of standard deviations
        return (Math.sqrt(varianceArea) + Math.sqrt(varianceAmbiguity)) / 2;
    }

    private static double getStandardDeviation(List<PhotonTrackedTarget> targets) {
        int count = 0;

        double meanArea = 0.0;
        double meanAmbiguity = 0.0;
        double sumSquareDiffArea = 0.0;
        double sumSquareDiffAmbiguity = 0.0;

        for (PhotonTrackedTarget photonTrackedTarget : targets) {
            count++;

            double deltaArea = photonTrackedTarget.getArea() - meanArea;
            meanArea += deltaArea / count;
            sumSquareDiffArea += deltaArea * (photonTrackedTarget.getArea() - meanArea);

            double deltaAmbiguity = photonTrackedTarget.getPoseAmbiguity() - meanAmbiguity;
            meanAmbiguity += deltaAmbiguity / count;
            sumSquareDiffAmbiguity += deltaAmbiguity * (photonTrackedTarget.getPoseAmbiguity() - meanAmbiguity);
        }
        
        if(count <= 1) {
            return 1e-4;
        } 

        double varianceArea = sumSquareDiffArea / (count - 1);
        double varianceAmbiguity = sumSquareDiffAmbiguity / (count - 1);

        return (Math.sqrt(varianceArea) + Math.sqrt(varianceAmbiguity)) / 2;
    }

    public ITranslation2d getInterpolatableTransform2d() {
        return new ITranslation2d(estimatedPose.getTranslation().toTranslation2d().getX(), estimatedPose.getTranslation().toTranslation2d().getY());
    }
}