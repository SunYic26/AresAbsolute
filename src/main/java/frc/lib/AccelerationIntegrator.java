package frc.lib;
import frc.lib.Interpolating.Geometry.ITwist2d;

import frc.robot.Subsystems.CommandSwerveDrivetrain.Drivetrain;

public class AccelerationIntegrator {

    private double prevXAccel = 0.0;
    private double prevYAccel = 0.0;

    private double xVelocity = 0;
    private double yVelocity = 0;

    private double lastTimestamp = 0;

    private double[] prevAccel = new double[2];
    
    private Drivetrain s_Swerve = Drivetrain.getInstance();

    public void initTimeStamp(double timestamp){
        lastTimestamp = timestamp;
    }

    private final double alpha = 0.50;  // TODO tune

    public void lowPassFilter(double[] accel) {
        accel[0] = alpha * accel[0] + (1-alpha) * prevAccel[0];
        accel[1] = alpha * accel[1] + (1-alpha) * prevAccel[1];
    }

    // Trapezoidal integration for velocity estimation (LIKE A BOSS)
    private void integrateAccel(double xAccel, double yAccel, double currentTime) {
        // if(lastTimestamp == -1) lastTimestamp = currentTime;

        if(Math.abs(xAccel) < 0.001) xAccel = 0;
        if(Math.abs(yAccel) < 0.001) yAccel = 0;

        double dt = currentTime - lastTimestamp;
        xVelocity += 0.5 * (xAccel + prevXAccel) * dt;
        yVelocity += 0.5 * (yAccel + prevYAccel) * dt;

        // Update prev
        prevXAccel = xAccel;
        prevYAccel = yAccel;
        lastTimestamp = currentTime;
    }

    public ITwist2d update(double[] accel) {

        lowPassFilter(accel);

        prevAccel = accel;

        integrateAccel(accel[0], accel[1], accel[2]);

        if(s_Swerve.getAbsoluteWheelVelocity() < 0.001){ //tune this threshold as needed
            xVelocity = 0;
            yVelocity = 0;
        } 

        return new ITwist2d(xVelocity, yVelocity);
    }

    public ITwist2d update(double[] accel, double[] wheelVelocity) {
        
        lowPassFilter(accel);

        prevAccel = accel;

        integrateAccel(accel[0], accel[1], accel[2]);

        if(s_Swerve.getAbsoluteWheelVelocity() < 0.001){ //tune this threshold as needed
            xVelocity = 0;
            yVelocity = 0;
        } 

        return new ITwist2d(xVelocity, yVelocity);
    }
}