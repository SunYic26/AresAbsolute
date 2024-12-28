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

    public static double MaxSpeed = 9; //can be lowered during testing
    public static double MaxAngularRate = 3 * Math.PI; // 3/4 of a rotation per second max angular velocity
    
    public static final int timeOutMs = 10;

    public static final double slipFactor = 65;
    public static final double slipThreshold = 0.15;

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

    public static class LimelightConstants{
        
        // TODO these are all from the example person but idk how many we will need and ofc they need to change for our design and new game (this is charged up i think)
        public static final int pipeNu_megatag = 0;
        public static final int pipeNu_node18_april = 1 ;
        public static final int pipeNu_node27_april = 2 ;
        public static final int pipeNu_node36_april = 3 ;
        public static final int pipeNu_hp45_april = 4;
        public static final int pipeNu_lower_reflective = 5;
        public static final int pipeNu_higher_reflective = 6;
        public static final int pipeNu_normal = 7;
        public static final double higher_reflective_heightCm = 111.0;
        public static final double lower_reflective_heightCm = 61.0;
        public static final double node_april_heightCm = 46.0;
        public static final double hp_april_heightCm = 69.0;
        public static final double limelightMountAngleDegrees = -19.5;
        public static final double limelightLensHeightCm = 107.0;

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

    //change for next game
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static final class FieldConstants {
        //constants about the field
    }

    public static final double FIELD_WIDTH_METERS = 8.21055;
    public static final double FIELD_LENGTH_METERS = 16.54175;

    public static final double openLoopRamp = 0.25;

    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }
}
