package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import choreo.Choreo;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystems.CommandSwerveDrivetrain.DriveControlSystems;
import frc.robot.generated.TunerConstants;
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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState.RobotState;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.CommandSwerveDrivetrain.DriveControlSystems;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import java.util.List;

import com.pathplanner.lib.path.*;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

public class FollowTrajectory extends Command {
    CommandSwerveDrivetrain s_Swerve;
    RobotState robotState;
    DriveControlSystems controlSystems;

    PathConstraints constraints = new PathConstraints(Constants.MaxSpeed, Constants.MaxAcceleration, Constants.MaxAngularRate, Constants.MaxAngularVelocity);
  
    PathPlannerTrajectory trajectory;
    List<Waypoint> poses;

    private Pose2d goalPose;

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

    SwerveDriveState state = s_Swerve.getState();

    // The rotation component of the pose should be the direction of travel
    poses = PathPlannerPath.waypointsFromPoses(
    new Pose2d(state.Pose.getTranslation(), new Rotation2d(Math.atan2(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond))),
    new Pose2d(0.5,0.5, new Rotation2d(0.0))
    );

    goalPose = new Pose2d(0.5, 0.5, new Rotation2d(0.0));

    GoalEndState endState = new GoalEndState(0.0, goalPose.getRotation());

    PathPlannerPath path = new PathPlannerPath(
    poses, 
    constraints, 
    null, //LEAVE THIS BLANK FOR ON THE FLY GENERATION BC NOT CONTROLLABLE IN TELEOP
    endState,
    false);

    timer.start();

    trajectory = path.generateTrajectory(state.Speeds, state.RawHeading, Constants.config);

    s_Swerve.resetOdo(trajectory.getInitialPose());
    }

    @Override
    public void execute() {
        
        PathPlannerTrajectoryState state = trajectory.sample(timer.get());

        System.out.println("x: "  + state.linearVelocity * state.heading.getCos());
        System.out.println("y: "  + state.linearVelocity * state.heading.getSin());
        System.out.println("ref: " + atReference());
        System.out.println("traj time: " + trajectory.getTotalTimeSeconds());
        System.out.println("timer: " + timer.hasElapsed(trajectory.getTotalTimeSeconds()));

        s_Swerve.setControl(
        new SwerveRequest.FieldCentric()
        .withVelocityX(state.linearVelocity * state.heading.getCos())
        .withVelocityY(state.linearVelocity * state.heading.getSin())
        .withRotationalRate(state.heading.getRadians()));
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
    }

    private boolean atReference() {
        Pose2d currPose2d = robotState.getCurrentPose2d();
        
        Pose2d diff = currPose2d.relativeTo(goalPose);

        if(Math.abs(diff.getX()) < Constants.TrajectoryConstants.poseToleranceX
         && Math.abs(diff.getY()) < Constants.TrajectoryConstants.poseToleranceY
         && (diff.getRotation().getRadians()) < Constants.TrajectoryConstants.poseToleranceTheta)
            return true;
         else
            return false;
    }

    @Override
    public boolean isFinished() {
        return atReference();
    }
}
