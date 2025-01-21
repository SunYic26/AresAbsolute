// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.CommandSwerveDrivetrain.DriveControlSystems;

import java.util.List;

import com.pathplanner.lib.path.*;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OnTheFlyGen extends Command {
  CommandSwerveDrivetrain s_Swerve;
  RobotState robotState;
  DriveControlSystems controlSystems;

  Pose2d goalPose;

  PathConstraints constraints = new PathConstraints(Constants.MaxSpeed, Constants.MaxAcceleration, Constants.MaxAngularRate, Constants.MaxAngularAcceleration);
  
  PathPlannerPath path;
  List<Waypoint> poses;

  Timer timer;

  public OnTheFlyGen(Pose2d goalPose) {
    this.s_Swerve = CommandSwerveDrivetrain.getInstance();
    this.controlSystems = DriveControlSystems.getInstance();
    this.robotState = RobotState.getInstance();

    this.goalPose = goalPose;
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    poses = PathPlannerPath.waypointsFromPoses(
    
  );

  path = new PathPlannerPath(
  poses, 
  constraints, 
  null, //LEAVE THIS BLANK FOR ON THE FLY GENERATION BC NOT CONTROLLABLE IN TELEOP
  
  );

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
