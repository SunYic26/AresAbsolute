package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystems.CommandSwerveDrivetrain.DriveControlSystems;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import edu.wpi.first.math.controller.LTVUnicycleController ;
import edu.wpi.first.math.controller.PIDController;
import frc.lib.Interpolating.Geometry.IPose2d;
import frc.lib.Interpolating.Geometry.ITranslation2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleSide;
import frc.robot.RobotState.RobotState;
import frc.robot.Constants.TrajectoryConstants;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;

public class FollowTrajectory extends Command {
    CommandSwerveDrivetrain s_Swerve;
    RobotState robotState;
    DriveControlSystems controlSystems;

    private Pose2d goalPose;

    Pose2d currentPose2D;

    Timer timer = new Timer();

    public FollowTrajectory(Pose2d goalPose) {
        this.s_Swerve = CommandSwerveDrivetrain.getInstance();
        this.controlSystems = DriveControlSystems.getInstance();
        this.robotState = RobotState.getInstance();

        this.goalPose = goalPose;
        addRequirements(s_Swerve);
    }

    public FollowTrajectory(ReefPoleSide side) {        
        this.s_Swerve = CommandSwerveDrivetrain.getInstance();
        this.controlSystems = DriveControlSystems.getInstance();
        this.robotState = RobotState.getInstance();

        this.goalPose = side.getClosestPoint(robotState.getCurrentPose2d());

        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {

        timer.start();

    }

    @Override
    public void execute() {
        // s_Swerve.setControl(
        // new SwerveRequest.FieldCentric()
        // .withVelocityX(controlOutput.vxMetersPerSecond * currPose.getRotation().getCos())
        // .withVelocityY(controlOutput.vxMetersPerSecond * currPose.getRotation().getSin())
        // .withRotationalRate(headingPidController.calculate(currPose.getRotation().getRadians(), trajectory.sample(timer.get()).poseMeters.getRotation().getRadians()) + controlOutput.omegaRadiansPerSecond));    
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
    }

    private boolean atReference() {
        // Pose2d currPose2d = robotState.getCurrentPose2d();
        // Pose2d goalPose2d = ;
        
        // Pose2d diff = currPose2d.relativeTo(goalPose2d);;

        // if(Math.abs(diff.getX()) < Constants.TrajectoryConstants.poseToleranceX
        //  && Math.abs(diff.getY()) < Constants.TrajectoryConstants.poseToleranceY
        //  && (diff.getRotation().getRadians()) < Constants.TrajectoryConstants.poseToleranceTheta)
        //     return true;
        //  else
            return false;
    }

    @Override
    public boolean isFinished() {
        return atReference();
    }
}
