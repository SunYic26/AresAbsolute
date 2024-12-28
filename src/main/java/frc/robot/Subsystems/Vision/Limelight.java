package frc.robot.Subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.CommandSwerveDrivetrain.Drivetrain;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase {
    private static Limelight instance;
    private NetworkTable nt;
    private boolean driverMode = false;

    private final Drivetrain drivetrain;

    
    // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    // NetworkTableEntry tx = table.getEntry("tx");
    // NetworkTableEntry ty = table.getEntry("ty");
    // NetworkTableEntry ta = table.getEntry("ta");

    public static Limelight getInstance() {
        if (instance == null) {
            instance = new Limelight();
        }
        return instance;
    }

    public enum LEDControl {
        LED_PipelineControlled(0),
        LED_Off(1),
        LED_Blink(2),
        LED_On(3);

        public int number() {
            return value;
        }

        int value;

        LEDControl(int value) {
            this.value = value;
        }
    }

    private Limelight() {

        nt = NetworkTableInstance.getDefault().getTable("limelight");
        drivetrain = Drivetrain.getInstance();
    }
    @AutoLogOutput(key = "Limelight/Connected")
    public boolean isConnected() {
        return NetworkTableInstance.getDefault().getTable("limelight").containsKey("ledMode");
    }

    @AutoLogOutput(key = "Limelight/hasTarget")
    public boolean hasTarget() {
       return nt.getEntry("tv").getDouble(0.0) == 1.0;
        // return LimelightHelpers.getTV("camera");
    }

    @AutoLogOutput(key = "Limelight/TargetArea")
    public double getTargetArea() {
       return nt.getEntry("ta").getDouble(0.0);
        // return LimelightHelpers.getTA("camera");
    }
    @AutoLogOutput(key = "Limelight/xOffset")
    public double getXOffset() {
       return nt.getEntry("tx").getDouble(0.0);
        // return LimelightHelpers.getTX("camera");
    }

    @AutoLogOutput(key = "Limelight/yOffset")
    public double getYOffset() {
       return nt.getEntry("ty").getDouble(0.0);
        // return LimelightHelpers.getTY("camera");
    }

    // can just use the LimelightHelpers.set whatever method for this, just call that method from basically anywhere
    public void setLEDMode(LEDControl mode) {
        nt.getEntry("ledMode").setNumber(mode.number());
    }
    
//    public LEDControl getLEDMode(){d
//        return nt.getEntry("ledMode").getInteger(-1);
//    }
    
    // note, cone cube etc
    public String getDetectorClass(){
        return LimelightHelpers.getDetectorClass("camera");
    }
    
    
    
    @Override
    public void periodic() {
        //read values periodically

        // System.out.println(nt);

        //post to smart dashboard periodically

       Logger.recordOutput("Limelight/xOffset", getXOffset());
       Logger.recordOutput("Limelight/yOffset", getYOffset());
    //    Logger.recordOutput("Limelight/Distance", getDistance());
    //    SmartDashboard.putNumber("LimelightX", getXOffset());
       SmartDashboard.putNumber("LimelightY", getYOffset());
//        SmartDashboard.putNumber("Limelight Distance", getDistance());

//        if(DriverStation.isTeleop()){
//
//            if(Math.abs(getXOffset()) < 20.0  && hasTarget()){      //&& Math.abs(getXOffset()) > 5.0
//                double x = 8.23 - (getDistance() *
//                        Math.cos(Math.toRadians(drivetrain.getGyroscopeRotation().getDegrees() + 180
//                                - getXOffset())));
//                double y = 4.165 - (getDistance() *
//                        Math.sin(Math.toRadians(drivetrain.getGyroscopeRotation().getDegrees() + 180
//                                - getXOffset())));//plus or minus xoffset???
//
//                drivetrain.resetOdometryFromPosition(x,y);
//            }
//        }
    }

    // @AutoLogOutput(key = "Limelight/Distance")
    // public double getDistance() {
    //     double limelightMountAngleDegrees = 27.0;
    //     double limelightLensHeightInches = 35;
    //     double goalHeightInches = 104.0;
    //     double angleToGoalDegrees = limelightMountAngleDegrees + getYOffset();

    //     //calculate distance
    //     double distanceFromLimelightToGoalInches =
    //             ((goalHeightInches - limelightLensHeightInches) / (Math.tan(Math.toRadians(angleToGoalDegrees))))
    //                     + 12 + 24;
    //     return distanceFromLimelightToGoalInches * 0.0254;
    // }
}