// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants.AprilTags;

import org.opencv.core.Point;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static double MaxSpeed = 12; //can be lowered during testing
    public static double MaxAngularRate = 3 * Math.PI; // 3/4 of a rotation per second max angular velocity
    
    public static final double slipFactor = 65;
    public static final double slipThreshold = 0.15;
    public static final int timeOutMs = 10;
    public static final double stickDeadband = 0.05;
    public static final double triggerDeadzone = 0.2;

    public static final class VisionConstants {
        public static final String cameraName = "camera";
        public static final int aprilTagMax = 16;
        public static final double aprilTagHeight = 0.122; //bottom of each april tag is 122cm above carpet | unnecessary, we have photonvision's field layout import
        public static final double cameraRollOffset = Units.degreesToRadians(0);
        public static final double cameraPitchOffset = Units.degreesToRadians(0);
        public static final double cameraYawOffset = Units.degreesToRadians(0);
        public static final double backRightCameraHeight = Units.inchesToMeters(9.1);
        public static final double backRightCameraPitch = Units.degreesToRadians(30);

        public static final double centerCameraHeight = Units.inchesToMeters(10.15);
        public static final double centerCameraPitch = Units.degreesToRadians(15);


        public static final class VisionLimits {
        public static final int k_rotationLimitDPS = 175;
        public static final double k_reprojectionLimit = 0.1;
        public static final double k_normThreshold = 0.1;
        public static final double k_ambiguityLimit = 0.9;

        }

        public static final class AprilTags {
            //fill with april tags
        }
    }


    // hardware ports for all hardware components on the robot
    // these include CAN IDs, pneumatic hub ports, etc.

    public static final class robotPIDs {
        //fill with our pids

        public static final class HeadingControlPID {
            public static final double highP = 12;
            public static final double highI = 0;
            public static final double highD = 4;

            public static final double lowP = 7;
            public static final double lowI = 0;
            public static final double lowD = 1.5;
        }

    }

    public static final class HardwarePorts {
        // motor id
    }



    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static final class FieldConstants {
        //constants about the field
    }

    /* Mechanism Current Limits */
    public static final int intakeContinuousCurrentLimit = 30;
    public static final int intakePeakCurrentLimit = 70;
    public static final int serializationContinuousCurrentLimit = 30;
    public static final int serializationPeakCurrentLimit = 70;
    public static final int ampContinuousCurrentLimit = 30;
    public static final int ampPeakCurrentLimit = 70;
    public static final int shooterContinuousCurrentLimit = 30; //commented out because the shooters stop too fast t
    public static final int shooterPeakCurrentLimit = 120;
    public static final int pivotContinuousCurrentLimit = 30;
    public static final int pivotPeakCurrentLimit = 60;
    public static final int climbContinuousCurrentLimit = 30; //arbitrary, fix later
    public static final int climbPeakCurrentLimit = 70; //arbitrary, fix later
    

    public static final double FIELD_WIDTH_METERS = 8.21055;
    public static final double FIELD_LENGTH_METERS = 16.54175;

    // public static final IdleMode intakeNeutralMode = IdleMode.kCoast;
    // public static final IdleMode shooterNeutralMode = IdleMode.kBrake;

    public static final double openLoopRamp = 0.25;

    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }
}
