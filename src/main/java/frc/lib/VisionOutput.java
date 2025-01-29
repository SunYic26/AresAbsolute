package frc.lib;

import java.util.Collections;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.Interpolating.Geometry.ITranslation2d;

/**
 * VisionOutput
 */
public class VisionOutput extends EstimatedRobotPose {

    public VisionOutput(Pose3d estimatedPose, double timestampSeconds, PhotonTrackedTarget target, PoseStrategy strategy)  {
        super(estimatedPose, timestampSeconds, Collections.singletonList(target), strategy);
    }

    public VisionOutput(EstimatedRobotPose pose)  {
        super(pose.estimatedPose, pose.timestampSeconds, pose.targetsUsed, pose.strategy);
    }

    //TODO
    // public VisionOutput(MultiTargetPNPResult multiTagResult)  {

    //     //transform the multitag to our frame of refrence, get all these values from the multitag

    //     super(pose.estimatedPose, pose.timestampSeconds, pose.targetsUsed, pose.strategy);
    // }

    static private double meanArea = 0.0;
    static private double meanAmbiguity = 0.0; //TODO get a default value for these but we need like a field or smth
    static private int count = 0;
    static private double sumSquareDiffArea = 0.0;
    static private double sumSquareDiffAmbiguity = 0.0;

    public double getStandardDeviation() {
        //TODO i want to do more here
        for (PhotonTrackedTarget photonTrackedTarget : this.targetsUsed) {
            count++;

            // Update means with Welford algorithm 
            double deltaArea = photonTrackedTarget.getArea() - meanArea;
            meanArea += deltaArea / count;
            sumSquareDiffArea += deltaArea * (photonTrackedTarget.getArea() - meanArea);

            double deltaAmbiguity = photonTrackedTarget.getPoseAmbiguity() - meanAmbiguity;
            meanAmbiguity += deltaAmbiguity / count;
            sumSquareDiffAmbiguity += deltaAmbiguity * (photonTrackedTarget.getPoseAmbiguity() - meanAmbiguity);
        }
        
        //dont divide by 0
        if(count <= 1) {
            return 1e-4;
        } 

        double varianceArea = sumSquareDiffArea / (count - 1);
        double varianceAmbiguity = sumSquareDiffAmbiguity / (count - 1);

        // Return average of standard deviations
        return (Math.sqrt(varianceArea) + Math.sqrt(varianceAmbiguity)) / 2;
    }

    public ITranslation2d getInterpolatableTransform2d() {
        return new ITranslation2d(estimatedPose.getTranslation().toTranslation2d().getX(), estimatedPose.getTranslation().toTranslation2d().getY());
    }



}