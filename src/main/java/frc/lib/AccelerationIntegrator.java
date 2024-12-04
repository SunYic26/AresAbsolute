package frc.lib;
import frc.lib.Interpolating.Geometry.Twist2d;

public class AccelerationIntegrator {
    private double prevAccel = 0.0;
    private double velocity = 0.0;
    private double lastTimestamp = 0.0;

    public void initTimeStamp(double timestamp){
        lastTimestamp = timestamp;
    }

    private double filteredAccel = 0.0;
    private final double alpha = 0.9;  // TODO tune

    public double lowPassFilter(double value) {
        // value = alpha * value + (1 - alpha) * filteredAccel;
        return value;
    }

    // Trapezoidal integration for velocity estimation (LIKE A BOSS)
    public double integrateAccel(double accel, double currentTime) {
        double dt = currentTime - lastTimestamp;
        velocity += 0.5 * (accel + prevAccel) * dt;

        // Update prev
        prevAccel = accel;
        lastTimestamp = currentTime;

        return velocity;
    }

    public Twist2d update(double x_AccelRaw, double y_AccelRaw, double currentTime) {
        double x_AccelFiltered = lowPassFilter(x_AccelRaw);  // Filter noisy input
        double y_AccelFiltered = lowPassFilter(y_AccelRaw);  // Filter noisy input
        return new Twist2d (integrateAccel(x_AccelFiltered, currentTime), integrateAccel(y_AccelFiltered, currentTime));  // Integrate for velocity
    }
}