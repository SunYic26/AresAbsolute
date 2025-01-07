// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import org.opencv.core.Point;
import org.opencv.core.Rect;

import frc.robot.RobotState.RobotState;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

/** Add your docs here. */
public class OuttakeProfiler {
    
    private static OuttakeProfiler instance;
    private RobotState robotState;


    private Point landingPoint;

    private Optional<DriverStation.Alliance> alliance;

    private Rect scoringArea; //change later, this will obviously not be a rectangle. using this for now for simplicity

    public static OuttakeProfiler getInstance(){
        if (instance == null) {
            instance = new OuttakeProfiler();
        }
        return instance;
    }

    private void calculateLandingPoint(){
    }

    private OuttakeProfiler() {
        robotState = RobotState.getInstance();
        alliance = DriverStation.getAlliance();
        if(alliance.get() == DriverStation.Alliance.Blue){
            scoringArea = new Rect();
        } else{
            scoringArea = new Rect();
        }
    }
    
    public boolean coralTrajAligned(){
        return landingPoint.inside(scoringArea);
    }
}
