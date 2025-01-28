package frc.lib.Interpolating.Geometry;

import frc.lib.Interpolating.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class IChassisSpeeds implements Interpolable<IChassisSpeeds> {
    protected final double vx;
    protected final double vy;
    protected final double omega;

    protected static final IChassisSpeeds kIdentity = new IChassisSpeeds(0.0, 0.0, 0.0);

    public static IChassisSpeeds identity() {
        return kIdentity;
    }

    public IChassisSpeeds(double vx, double vy, double omega) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }

    public IChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        this.vx = chassisSpeeds.vxMetersPerSecond;
        this.vy = chassisSpeeds.vyMetersPerSecond;
        this.omega = chassisSpeeds.omegaRadiansPerSecond;
    }

    public IChassisSpeeds(final IChassisSpeeds other) {
        this.vx = other.vx;
        this.vy = other.vy;
        this.omega = other.omega;
    }

    public double getVx() {
        return vx;
    }

    public double getVy() {
        return vy;
    }

    public double getOmega() {
        return omega;
    }

    public double toMagnitude() {
        return Math.hypot(this.vx, this.vy);
    }

    @Override
    public IChassisSpeeds interpolate(IChassisSpeeds other, double x) {
        if (x <= 0) {
            return new IChassisSpeeds(this);
        } else if (x >= 1) {
            return new IChassisSpeeds(other);
        }
        return new IChassisSpeeds(
            this.vx + (other.vx - this.vx) * x,
            this.vy + (other.vy - this.vy) * x,
            this.omega + (other.omega - this.omega) * x);
    }
 }
