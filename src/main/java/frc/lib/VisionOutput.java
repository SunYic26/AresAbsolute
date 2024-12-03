package frc.lib;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import org.opencv.photo.Photo;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.Interpolating.Geometry.*;

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

    static private double meanArea = 0.0;
    static private double meanAmbiguity = 0.0; //TODO get a default value for these but we need like a field or smth
    static private int count = 0;
    static private double sumSquareDiffArea = 0.0;
    static private double sumSquareDiffAmbiguity = 0.0;

    // public void updateStatistics() {
    // }

    public double getStandardDeviation() {
        //TODO i want to do more here
        for (PhotonTrackedTarget photonTrackedTarget : this.targetsUsed) {
            count++;

            // Update means wit Welford algorithm 
            double deltaArea = photonTrackedTarget.getArea() - meanArea;
            meanArea += deltaArea / count;
            sumSquareDiffArea += deltaArea * (photonTrackedTarget.getArea() - meanArea);

            double deltaAmbiguity = photonTrackedTarget.getPoseAmbiguity() - meanAmbiguity;
            meanAmbiguity += deltaAmbiguity / count;
            sumSquareDiffAmbiguity += deltaAmbiguity * (photonTrackedTarget.getPoseAmbiguity() - meanAmbiguity);
        }

        if(count <= 1) {
            return 0;
        } //dont divide by 0 brah

        double varianceArea = sumSquareDiffArea / (count - 1);
        double varianceAmbiguity = sumSquareDiffAmbiguity / (count - 1);

        // Return average of standard deviations
        return (Math.sqrt(varianceArea) + Math.sqrt(varianceAmbiguity)) / 2;
    }

    public ITranslation2d getInterpolatableTransform2d() {
        return new ITranslation2d(estimatedPose.getTranslation().toTranslation2d().getX(), estimatedPose.getTranslation().toTranslation2d().getY());
    }



}