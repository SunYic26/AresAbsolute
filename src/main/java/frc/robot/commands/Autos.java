// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.CommandSwerveDrivetrain.Drivetrain;
import frc.robot.Subsystems.Vision.Vision;

/** Add your docs here. */
public class Autos {
    private RobotContainer m_robotContainer;

    private final Vision vision = Vision.getInstance();

    private static Drivetrain drivetrain = Drivetrain.getInstance();
    private static RobotState robotState = RobotState.getInstance();
    private static final PIDController thetaController = new PIDController(3, 1.4, 0); //tune?
    private static final PIDController xController = new PIDController(5, 1, 0);
    private static final PIDController yController = new PIDController(5, 1, 0);


    public static Command FollowChoreoTrajectory(ChoreoTrajectory path) {
        SwerveRequest.ApplyChassisSpeeds drive = new SwerveRequest.ApplyChassisSpeeds();
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        ChoreoTrajectory traj = path;
        thetaController.reset();
        xController.reset();
        yController.reset();

        drivetrain.setAutoStartPose(traj.getInitialPose()); //NOTE: THIS MAY BE BROKEN, BE READY WHEN FIRST TESTING AUTO TO DISABLE
        SmartDashboard.putNumber("Start pose x", traj.getInitialPose().getX());
        Command swerveCommand = Choreo.choreoSwerveCommand(
                traj,
                drivetrain::getPose,
                xController,
                yController,
                thetaController,
                (ChassisSpeeds speeds) -> drivetrain.setControl(drive.withSpeeds(speeds)),
                // () -> {return false;},
                // (ChassisSpeeds speeds) -> s_Swerve.applyRequest(() -> drive.withSpeeds(speeds)),
                () -> {
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == Alliance.Red;
                },
                drivetrain);

        return swerveCommand;
    }

  public static Command Test1() {
    return Commands.waitSeconds(0);
  }

  public static Command Test2() {
    return Commands.waitSeconds(0);
  }

  public static Command Test3() {
    return Commands.waitSeconds(0);
  }
  
  public static Command Test4() {
    return Commands.waitSeconds(0);
  }
}
