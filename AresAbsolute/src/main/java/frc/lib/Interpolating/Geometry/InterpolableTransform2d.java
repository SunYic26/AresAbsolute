package frc.lib.Interpolating.Geometry;

import frc.lib.Interpolating.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class InterpolableTransform2d implements Interpolable<InterpolableTransform2d> {

    protected static final InterpolableTransform2d kIdentity = new InterpolableTransform2d();

    public static InterpolableTransform2d identity() {
        return kIdentity;
    }

    protected final double x_;
    protected final double y_;

    public InterpolableTransform2d() {
        x_ = 0;
        y_ = 0;
    }

    public InterpolableTransform2d(double x, double y) {
        x_ = x;
        y_ = y;
    }

    public InterpolableTransform2d(final InterpolableTransform2d other) {
        x_ = other.x_;
        y_ = other.y_;
    }

    public InterpolableTransform2d(final InterpolableTransform2d start, final InterpolableTransform2d end) {
        x_ = end.x_ - start.x_;
        y_ = end.y_ - start.y_;
    }

    public InterpolableTransform2d(final edu.wpi.first.math.geometry.Translation2d other) {
        x_= other.getX();
        y_ = other.getY();
    }

        
    public static InterpolableTransform2d fromPolar(Rotation2d direction, double magnitude){
    	return new InterpolableTransform2d(direction.getCos() * magnitude, direction.getSin() * magnitude);
    }

    /**
     * The "norm" of a transform is the Euclidean distance in x and y.
     *
     * @return sqrt(x ^ 2 + y ^ 2)
     */
    public double norm() {
        return Math.hypot(x_, y_);
    }

    public double norm2() {
        return x_ * x_ + y_ * y_;
    }

    public double x() {
        return x_;
    }

    public double y() {
        return y_;
    }


        @Override
    public InterpolableTransform2d interpolate(final InterpolableTransform2d other, double x) {
        if (x <= 0) {
            return new InterpolableTransform2d(this);
        } else if (x >= 1) {
            return new InterpolableTransform2d(other);
        }
        return extrapolate(other, x);
    }

    public InterpolableTransform2d extrapolate(final InterpolableTransform2d other, double x) {
        return new InterpolableTransform2d(x * (other.x_ - x_) + x_, x * (other.y_ - y_) + y_);
    }
}
