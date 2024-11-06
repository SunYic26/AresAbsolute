package frc.lib.Interpolating.Geometry;


/**
 * velocityTwist2d
 */
public class Twist2d {
    /** Linear "dx" component. */
    private double dx;

    /** Linear "dy" component. */
    private double dy;

    public Twist2d(double dx, double dy) {
        this.dx = dx;
        this.dy = dy;
    }    

    public double getX() {
        return dx;
    }

    public double getY() {
        return dy;
    }
}