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

    private static double meanArea = 0.0;
    private static double meanAmbiguity = 0.0; //TODO get a default value for these but we need like a field or smth
    private static int count = 0;

    public double getStandardDeviation() { //did i do this right
        double varArea = 0, varAmbiguity = 0;
        for (PhotonTrackedTarget photonTrackedTarget : this.targetsUsed) {
            count++;
            varArea += Math.pow(photonTrackedTarget.getArea() 
                - (meanArea += photonTrackedTarget.getArea())/count, 2);
            varAmbiguity += Math.pow(photonTrackedTarget.getPoseAmbiguity()
                - (meanAmbiguity += photonTrackedTarget.getPoseAmbiguity())/count, 2);
        }

        return (Math.sqrt(varAmbiguity) + Math.sqrt(varArea))/2;
    }

    public ITranslation2d getInterpolatableTransform2d() {
        return new ITranslation2d(estimatedPose.getTranslation().toTranslation2d().getX(), estimatedPose.getTranslation().toTranslation2d().getY());
    }



}