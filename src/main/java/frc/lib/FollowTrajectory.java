package frc.lib;

import java.util.List;
import java.util.function.Supplier;

import choreo.Choreo;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystems.CommandSwerveDrivetrain.DriveControlSystems;
import frc.robot.Subsystems.CommandSwerveDrivetrain.Drivetrain;
import edu.wpi.first.math.controller.LTVUnicycleController ;
import frc.lib.Interpolating.Geometry.ITranslation2d;
import frc.robot.Constants.FieldConstants.Reef.ReefPoleSide;
import frc.robot.RobotState.RobotState;

public class FollowTrajectory extends Command {
    Drivetrain s_Swerve;
    RobotState robotState;
    DriveControlSystems controlSystems;

    private Pose2d goalPose;
    private Trajectory trajectory;

    Pose2d currentPose2D;

    //TODO change these
    TrajectoryConfig config = new TrajectoryConfig(2.0, 1.0); 

    private LTVUnicycleController ltvUnicycleController = new LTVUnicycleController(0.02, 6); // Ramsete tuning constants
    Timer timer = new Timer();

    public FollowTrajectory(Pose2d goalPose) {
        this.s_Swerve = Drivetrain.getInstance();
        this.controlSystems = DriveControlSystems.getInstance();
        this.robotState = RobotState.getInstance();
        this.goalPose = goalPose;
        addRequirements(s_Swerve);
    }

    public FollowTrajectory(ReefPoleSide side) {        
        this.s_Swerve = Drivetrain.getInstance();
        this.controlSystems = DriveControlSystems.getInstance();
        this.robotState = RobotState.getInstance();
        this.goalPose = side.getClosestPoint(robotState.getCurrentPose2d());
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        s_Swerve.resetOdo(trajectory.getInitialPose());
        ltvUnicycleController.setTolerance(new Pose2d(0.05, 0.03, new Rotation2d(0.1))); 

        timer.start();
        trajectory = TrajectoryGenerator.generateTrajectory(
            List.of(currentPose2D, goalPose), config
        );
    }

    @Override
    public void execute() {
        ChassisSpeeds controlOutput = ltvUnicycleController.calculate(robotState.getCurrentPose2d(), trajectory.sample(timer.get()));
        s_Swerve.trajectoryDrive(controlOutput.vxMetersPerSecond, controlOutput.vyMetersPerSecond, controlOutput.omegaRadiansPerSecond);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return ltvUnicycleController.atReference() || (timer.hasElapsed(0.1) && controlSystems.pollInput());
    }
}
