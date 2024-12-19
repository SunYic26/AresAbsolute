package frc.robot.Subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import org.littletonrobotics.junction.Logger;

// import frc.robot.RobotState.RobotState;

public class DriveControlSystems {

    private boolean slipControlOn = false;
    private boolean headingControl = false;
    private boolean shooterMode = false;
    private boolean aligning = false;
    private double lastHeading = 0;

    // Can tune
    private double deadbandFactor = 0.8; // higher is more linear joystick controls


    Drivetrain drivetrain;
    // RobotState robotState;

    PIDController pidHeading = new PIDController(0, 0, 0);

    public DriveControlSystems() {  
        // robotState = RobotState.getInstance();
        drivetrain = Drivetrain.getInstance();
    }

    //interface with modules
    public SwerveModule getModule(int index) {
      return drivetrain.getModule(index);
    }   

     // =======---===[ ⚙ Joystick processing ]===---========
    public SwerveRequest drive(double driverLY, double driverLX, double driverRX){
        driverLX = scaledDeadBand(driverLX) * Constants.MaxSpeed;
        driverLY = scaledDeadBand(driverLY) * Constants.MaxSpeed;
        driverRX = scaledDeadBand(driverRX) * Constants.MaxAngularRate;

        //heading control
        // if (headingControl && driverRX < 0.1) {
        //     driverRX = headingControl(driverRX);
        // }

        // //slip control
        // if (slipControlOn) {
        // slipCorrection(slipControl(drivetrain.robotAbsoluteVelocity())); 
        // }

//        SmartDashboard.putNumber("requested velocity x", driverLX);
//        SmartDashboard.putNumber("requested velocity y", driverLY);
        Logger.recordOutput("JoystickProcessing/RequestedX", driverLX);
        Logger.recordOutput("JoystickProcessing/RequestedY", driverLY);
        
        return new SwerveRequest.FieldCentric()
        .withVelocityX(driverLY)
        .withVelocityY(driverLX)
        .withRotationalRate(driverRX)
        .withDeadband(Constants.MaxSpeed * RobotContainer.translationDeadband)
        .withRotationalDeadband(Constants.MaxAngularRate * RobotContainer.rotDeadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    }

    public double scaledDeadBand(double input) {
        return (deadbandFactor * Math.pow(input, 3)) + (1 - deadbandFactor) * input;
    }

    // =======---===[ ⚙ Heading control ]===---========
    // public double headingControl(double driverRX){ //TODO tune high and low PID values
    //     if (!pidHeading.atSetpoint()) {
    //         double velocity = drivetrain.robotWheelVelocity();
    //         updateGains(velocity);
            
    //         // driverRX = pidHeading.calculate(robotState.robotYaw(), lastHeading);
    //         SmartDashboard.putBoolean("headingON", true);
//    Logger.recordOutput("HeadingControl/Active", true);

    //     } else {
    //         SmartDashboard.putBoolean("headingON", false);
    //         SmartDashboard.putNumber("lastHeading", lastHeading);
//    Logger.recordOutput("HeadingControl/Active", false);
//    Logger.recordOutput("HeadingControl/LastHeading", lastHeading);
    //     }

    //     return driverRX;
    // } 
    // TODO fix this later bruh

    public void updateGains(double velocity) {
        double speedRatio = Math.abs(Constants.MaxSpeed/velocity); //velocity is from wheels so could be off
        speedRatio = Math.max(0, Math.min(1, speedRatio));
        //clamp between 0 and 1

        //can tune
        pidHeading.setPID(
            interpolate(Constants.robotPIDs.HeadingControlPID.lowP, Constants.robotPIDs.HeadingControlPID.highP, speedRatio), // P
            0, // I (we do not need I)
            interpolate(Constants.robotPIDs.HeadingControlPID.lowD, Constants.robotPIDs.HeadingControlPID.highD, speedRatio) // D
            ); 
    }

    public double interpolate(double lower, double upper, double scale) {
        return Interpolator.forDouble().interpolate(lower, upper, scale);
    }

    // =======---===[ ⚙ Slip Control ]===---========
    public Double[] slipControl(double currentVelocity) {

    Double[] outputs = new Double[4]; // reset to null every call
//    SmartDashboard.putNumber("currentVelocity", currentVelocity);
        Logger.recordOutput("currentVelocity", currentVelocity);
        for (int i = 0; i < 4; i++) {  //4 is module count but i dont want to make a getter
        
        //gets the ratio between what the encoders think our velocity is and the real velocity
        double slipRatio;
        if(currentVelocity == 0) { slipRatio = 1; } else {
            slipRatio = ((getModule(i).getCurrentState().speedMetersPerSecond) / currentVelocity); 
        }
//        SmartDashboard.putNumber("Module " + i + " slipratio", slipRatio);
        Logger.recordOutput("SwerveModules/SlipRatios/Module " + i , slipRatio);
        //if over the upper or lower threshold save the value
        if (slipRatio > (Constants.slipThreshold + 1) || slipRatio < (1 - Constants.slipThreshold)) {
            outputs[i] = slipRatio;
        }
    }

    return outputs;
    } // runs periodically as a default command

    public void slipCorrection(Double[] inputs) {
        // divides by slip factor, more aggressive if far above slip threshold 
        for (int i = 0; i < 4; i++) { //4 is module count but i dont want to make a getter

            if (inputs[i] != null) {
                TalonFX module = getModule(i).getDriveMotor();
                
                module.set(module.get() *
                 (1 + (Math.signum(inputs[i] - 1)) * (inputs[i] - Constants.slipThreshold)) / Constants.slipFactor);
                //https://www.desmos.com/calculator/afe5omf92p how slipfactor changes slip aggression

//                SmartDashboard.putBoolean("slipON", true);
                Logger.recordOutput("SlipControl/Active", true);
            }  else {
//                SmartDashboard.putBoolean("slipON", false);
                Logger.recordOutput("SlipControl/Active", false);
            } 
            
        }
    }

    //toggling ----------------------------------------------------------------
    public void setLastHeading() {
        lastHeading = drivetrain.getPose().getRotation().getRadians(); 
    }

    public void toggleHeadingControl() {
        headingControl = !headingControl;
    }

    public void toggleAlignment() {
        aligning = !aligning;   
    }

    public void toggleSlipControl() {
        slipControlOn = !slipControlOn;
    }

    public void setHeadingTolerance() {
        pidHeading.setTolerance(0.1745); // 10 degrees in radians
    }


}
