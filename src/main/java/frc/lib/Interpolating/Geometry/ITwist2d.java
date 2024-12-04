package frc.lib.Interpolating.Geometry;

import frc.lib.Interpolating.Interpolable;

/**
 * velocityTwist2d
 */
public class ITwist2d implements Interpolable<ITwist2d> {

    protected static final ITwist2d kIdentity = new ITwist2d();

    public static ITwist2d identity() {
        return kIdentity;
    }

    protected final double dx;
    protected final double dy;

    public ITwist2d() {
        this.dx = 0;
        this.dy = 0;
    }  

    public ITwist2d(double dx, double dy) {
        this.dx = dx;
        this.dy = dy;
    }

    public ITwist2d(final ITwist2d other) {
        dx = other.dx;
        dy = other.dy;
    }


    public double getX() {
        return dx;
    }

    public double getY() {
        return dy;
    }

    @Override
    public ITwist2d interpolate(final ITwist2d other, double x) {
        if (x <= 0) {
            return new ITwist2d(this);
        } else if (x >= 1) {
            return new ITwist2d(other);
        }
        return extrapolate(other, x);
    }

    public ITwist2d extrapolate(final ITwist2d other, double x) {
        return new ITwist2d(x * (other.dx - dx) + dx, x * (other.dy - dy) + dy);
    }

}