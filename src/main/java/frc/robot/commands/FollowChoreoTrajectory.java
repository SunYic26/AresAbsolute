// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import choreo.Choreo;
import choreo.trajectory.Trajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CommandSwerveDrivetrain.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FollowChoreoTrajectory extends Command {
  private final Trajectory trajectory;
  private final Drivetrain s_Swerve;
  private DriverStation.Alliance alliance;
  private Pose2d startPose;
  private Timer timer;
  public FollowChoreoTrajectory(String name) {
    trajectory = Choreo.loadTrajectory(name).get();
    s_Swerve = Drivetrain.getInstance();
    alliance = DriverStation.getAlliance().get();
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    startPose = trajectory.getInitialPose(alliance == DriverStation.Alliance.Red).get();
    s_Swerve.resetOdo(startPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveSample sample = trajectory.sampleAt(timer.get(), alliance == DriverStation.Alliance.Red);
    s_Swerve.followAutoTrajectory(sample);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= trajectory.getTotalTime();
  }
}
