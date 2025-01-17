// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.Trajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FollowChoreoTrajectory extends Command {
  private final Trajectory trajectory;
  private final CommandSwerveDrivetrain s_Swerve;
  private Optional<DriverStation.Alliance> alliance;
  private Optional<Pose2d> startPose;
  private Timer timer;
  public FollowChoreoTrajectory(String name) {
    trajectory = Choreo.loadTrajectory(name).get();
    s_Swerve = CommandSwerveDrivetrain.getInstance();
    alliance = DriverStation.getAlliance();
    timer = new Timer();
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    startPose = trajectory.getInitialPose(alliance.get() == DriverStation.Alliance.Red);
    s_Swerve.resetOdo(startPose.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<SwerveSample> sample = trajectory.sampleAt(timer.get(), alliance.get() == DriverStation.Alliance.Red);
    // s_Swerve.followAutoTrajectory(sample.get());
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
