// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import java.awt.Point;
import frc.lib.Hexagon;
import frc.robot.Constants;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.Outtake;
import frc.robot.Subsystems.CommandSwerveDrivetrain.Drivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

/** Add your docs here. */
public class OuttakeProfiler {
    
    private static OuttakeProfiler instance;
    private RobotState robotState;
    private Outtake s_Outtake;
    private Drivetrain s_Swerve;
    private Pose2d robotPose;

    private double horizontalVel;
    private double verticalVel;

    private double outtakeDirection;

    private Point landingPoint;

    private Optional<DriverStation.Alliance> alliance;

    private Hexagon scoringArea; //change later, this will obviously not be a rectangle. using this for now for simplicity
    private Hexagon innerArea;

    public static OuttakeProfiler getInstance(){
        if (instance == null) {
            instance = new OuttakeProfiler();
        }
        return instance;
    }

    private void calculateLandingPoint(){
        horizontalVel = Math.cos(Math.toRadians(Constants.outtakeAngle))*s_Outtake.getOutputSpeed();
        verticalVel = Math.sin(Math.toRadians(Constants.outtakeAngle))*s_Outtake.getOutputSpeed();

        //horizontalVel += s_Swerve.getWheelVelocities()[0];
        //verticalVel += s_Swerve.getWheelVelocities()[1];

        robotPose = s_Swerve.getPose();

        outtakeDirection = Math.toDegrees(s_Swerve.getHeading());

    }

    private OuttakeProfiler() {
        robotState = RobotState.getInstance();
        alliance = DriverStation.getAlliance();
        s_Outtake = Outtake.getInstance();
        if(alliance.get() == DriverStation.Alliance.Blue){
            scoringArea = new Hexagon(new Point(), 0.75); //gave about 15 cm of clearance, actual sidelength is 0.9
        } else{
            scoringArea = new Hexagon(new Point(), 0.75); //TODO: get coordinates of center of reef and use them here
        }

        if(alliance.get() == DriverStation.Alliance.Blue){
            innerArea = new Hexagon(new Point(), 0.55);
        } else{
            innerArea = new Hexagon(new Point(), 0.55);
        }
    }
    
    public boolean coralTrajAligned(){
        return scoringArea.contains(landingPoint) && !innerArea.contains(landingPoint);
    }
}
