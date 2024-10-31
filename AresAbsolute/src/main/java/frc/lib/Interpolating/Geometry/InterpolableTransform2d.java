package frc.lib.Interpolating.Geometry;

import frc.lib.Interpolating.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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

    public InterpolableTransform2d(Transform2d transform) {
        this.x_ = transform.getX();
        this.y_ = transform.getY();
    }

    public InterpolableTransform2d(InterpolablePose2d pose) {
        this.x_ = pose.getX();
        this.y_ = pose.getY();
    }

    public InterpolableTransform2d(Pose2d pose) {
        this.x_ = pose.getX();
        this.y_ = pose.getY();
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

    public double getX() {
        return x_;
    }

    public double getY() {
        return y_;
    }

    public InterpolableTransform2d translateBy(final InterpolableTransform2d other) {
        return new InterpolableTransform2d(x_ + other.x_, y_ + other.y_);
    }

    public InterpolableTransform2d plus(InterpolableTransform2d other) {
        return new InterpolableTransform2d(x_ + other.getX(), y_ + other.getY());
    }

    public InterpolableTransform2d minus(InterpolableTransform2d other) {
        return new InterpolableTransform2d(x_ - other.getX(), y_ - other.getY());
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
