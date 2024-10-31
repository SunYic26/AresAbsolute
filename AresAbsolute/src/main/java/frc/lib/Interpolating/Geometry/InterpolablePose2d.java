package frc.lib.Interpolating.Geometry;

import frc.lib.Interpolating.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class InterpolablePose2d implements Interpolable<InterpolablePose2d> {
    private final double x;
    private final double y;
    private final Rotation2d rotation;

     protected static final InterpolablePose2d kIdentity = new InterpolablePose2d(0.0, 0.0, new Rotation2d());

    public static InterpolablePose2d identity() {
        return kIdentity;
    }

    public InterpolablePose2d(double x, double y, Rotation2d rotation) {
        this.x = x;
        this.y = y;
        this.rotation = rotation;
    }

    public InterpolablePose2d(Pose2d pose) {
        this.x = pose.getX();
        this.y = pose.getY();
        this.rotation = pose.getRotation();
    }

    @Override
    public InterpolablePose2d interpolate(InterpolablePose2d other, double ratio) {
        if (ratio < 0 || ratio > 1) {
            throw new IllegalArgumentException("Ratio must be between 0 and 1");
        }
        
        // Interpolate x, y, and rotation
        double newX = this.x + (other.x - this.x) * ratio;
        double newY = this.y + (other.y - this.y) * ratio;
        Rotation2d newRotation = this.rotation.interpolate(other.rotation, ratio);

        return new InterpolablePose2d(newX, newY, newRotation);
    }

    // Getters for x, y, rotation if needed
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public Rotation2d getRotation() {
        return rotation;
    }
}
