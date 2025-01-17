package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import choreo.Choreo;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystems.CommandSwerveDrivetrain.DriveControlSystems;
import frc.robot.Subsystems.CommandSwerveDrivetrain.CommandSwerveDrivetrain;
import edu.wpi.first.math.controller.LTVUnicycleController ;
import frc.lib.Interpolating.Geometry.ITranslation2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefPoleSide;
import frc.robot.RobotState.RobotState;
import frc.robot.Constants.TrajectoryConstants;

public class FollowTrajectory extends Command {
    CommandSwerveDrivetrain s_Swerve;
    RobotState robotState;
    DriveControlSystems controlSystems;

    private Pose2d goalPose;
    private Trajectory trajectory;

    Pose2d currentPose2D;

    //TODO change these
    TrajectoryConfig config = new TrajectoryConfig(
                                            Constants.TrajectoryConstants.maxVelocity,
                                            Constants.TrajectoryConstants.maxAcceleration); 

    private LTVUnicycleController ltvUnicycleController = new LTVUnicycleController(0.02, Constants.TrajectoryConstants.maxVelocity); // Ramsete tuning constants
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
        ltvUnicycleController.setTolerance(new Pose2d(
                                            Constants.TrajectoryConstants.poseToleranceX,
                                            Constants.TrajectoryConstants.poseToleranceY,
                              new Rotation2d(Constants.TrajectoryConstants.poseToleranceTheta))); 

        timer.start();
        trajectory = TrajectoryGenerator.generateTrajectory(
            List.of(robotState.getCurrentPose2d(), goalPose), config
        );

        s_Swerve.resetOdo(trajectory.getInitialPose());
    }

    @Override
    public void execute() {

        ChassisSpeeds controlOutput = ltvUnicycleController.calculate(robotState.getCurrentPose2d(), trajectory.sample(timer.get()));
        // s_Swerve.trajectoryDrive(controlOutput.vxMetersPerSecond, controlOutput.vyMetersPerSecond, controlOutput.omegaRadiansPerSecond);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return ltvUnicycleController.atReference();
    }// timer check is so we dont accidentally cancel at the start of the command
}
