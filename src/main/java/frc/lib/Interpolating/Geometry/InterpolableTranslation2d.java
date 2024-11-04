package frc.lib.Interpolating.Geometry;

import frc.lib.Interpolating.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class InterpolableTranslation2d implements Interpolable<InterpolableTranslation2d> {

    protected static final InterpolableTranslation2d kIdentity = new InterpolableTranslation2d();

    public static InterpolableTranslation2d identity() {
        return kIdentity;
    }

    protected final double x_;
    protected final double y_;

    public InterpolableTranslation2d() {
        x_ = 0;
        y_ = 0;
    }

    public InterpolableTranslation2d(Transform2d transform) {
        this.x_ = transform.getX();
        this.y_ = transform.getY();
    }

    public InterpolableTranslation2d(InterpolablePose2d pose) {
        this.x_ = pose.getX();
        this.y_ = pose.getY();
    }

    public InterpolableTranslation2d(Pose2d pose) {
        this.x_ = pose.getX();
        this.y_ = pose.getY();
    }


    public InterpolableTranslation2d(double x, double y) {
        x_ = x;
        y_ = y;
    }

    public InterpolableTranslation2d(final InterpolableTranslation2d other) {
        x_ = other.x_;
        y_ = other.y_;
    }

    public InterpolableTranslation2d(final InterpolableTranslation2d start, final InterpolableTranslation2d end) {
        x_ = end.x_ - start.x_;
        y_ = end.y_ - start.y_;
    }

    public InterpolableTranslation2d(final edu.wpi.first.math.geometry.Translation2d other) {
        x_= other.getX();
        y_ = other.getY();
    }

        
    public static InterpolableTranslation2d fromPolar(Rotation2d direction, double magnitude){
    	return new InterpolableTranslation2d(direction.getCos() * magnitude, direction.getSin() * magnitude);
    }

    public double getX() {
        return x_;
    }

    public double getY() {
        return y_;
    }

    public InterpolableTranslation2d translateBy(final InterpolableTranslation2d other) {
        return new InterpolableTranslation2d(x_ + other.x_, y_ + other.y_);
    }

    /**
     * @param other translation2d to merge in
     * @param weight is the trust in the passed in current, from 0 - 1
     */
    public InterpolableTranslation2d weightedAverageBy(final InterpolableTranslation2d other, double weight) {
        return new InterpolableTranslation2d(x_* (weight) + other.getX() * (1-weight),
                                             y_* (weight) + other.getY() * (1-weight));
    }

    public InterpolableTranslation2d plus(InterpolableTranslation2d other) {
        return new InterpolableTranslation2d(x_ + other.getX(), y_ + other.getY());
    }

    public InterpolableTranslation2d minus(InterpolableTranslation2d other) {
        return new InterpolableTranslation2d(x_ - other.getX(), y_ - other.getY());
    }

        @Override
    public InterpolableTranslation2d interpolate(final InterpolableTranslation2d other, double x) {
        if (x <= 0) {
            return new InterpolableTranslation2d(this);
        } else if (x >= 1) {
            return new InterpolableTranslation2d(other);
        }
        return extrapolate(other, x);
    }

    public InterpolableTranslation2d extrapolate(final InterpolableTranslation2d other, double x) {
        return new InterpolableTranslation2d(x * (other.x_ - x_) + x_, x * (other.y_ - y_) + y_);
    }
}
