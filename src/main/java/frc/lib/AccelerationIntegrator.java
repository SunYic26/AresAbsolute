package frc.lib;
import frc.lib.Interpolating.Geometry.ITwist2d;

import frc.robot.Subsystems.CommandSwerveDrivetrain.Drivetrain;

public class AccelerationIntegrator {
    private double prevXAccel = 0.0;
    private double prevYAccel = 0.0;
    private ITwist2d velocity;
    private double xVelocity = 0;
    private double yVelocity = 0;
    private double lastTimestamp = 0;

    private Drivetrain s_Swerve = Drivetrain.getInstance();

    public void initTimeStamp(double timestamp){
        lastTimestamp = timestamp;
    }

    private double xFilteredAccel = 0.0;
    private double yFilteredAccel = 0.0;
    private final double alpha = 0.70;  // TODO tune

    public double xLowPassFilter(double value) {
        value = alpha * value + (1 - alpha) * xFilteredAccel;
        return value;
    }
    
    public double yLowPassFilter(double value) {
        value = alpha * value + (1 - alpha) * yFilteredAccel;
        return value;
    }

    // Trapezoidal integration for velocity estimation (LIKE A BOSS)
    public void integrateAccel(double xAccel, double yAccel, double currentTime) {
        // if(lastTimestamp == -1) lastTimestamp = currentTime;

        if(Math.abs(xAccel) < 0.002) xAccel = 0;
        if(Math.abs(yAccel) < 0.002) yAccel = 0;

        double dt = currentTime - lastTimestamp;
        xVelocity += 0.5 * (xAccel + prevXAccel) * dt;
        yVelocity += 0.5 * (yAccel + prevYAccel) * dt;

        // Update prev
        prevXAccel = xAccel;
        prevYAccel = yAccel;
        lastTimestamp = currentTime;
    }

    public ITwist2d update(double x_AccelRaw, double y_AccelRaw, double currentTime) {
        double x_AccelFiltered = xLowPassFilter(x_AccelRaw);  // Filter noisy input
        double y_AccelFiltered = yLowPassFilter(y_AccelRaw);  // Filter noisy input
        integrateAccel(x_AccelFiltered, y_AccelFiltered, currentTime);
        if(s_Swerve.getWheelVelocity() < 0.001){ //tune this threshold as needed
            xVelocity = 0;
            yVelocity = 0;
        } 
        return new ITwist2d(xVelocity, yVelocity); //double (integrateAccel(x_AccelFiltered, currentTime), integrateAccel(y_AccelFiltered, currentTime));  // Integrate for velocity
    }
}