package frc.robot.RobotState;

import java.io.ObjectInputStream.GetField;
import java.util.List;
import java.util.Optional;
import java.util.function.BiFunction;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.interpolation.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.UnitBuilder;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
import frc.lib.Interpolating.Geometry.IPose2d;
import frc.lib.Interpolating.Geometry.ITranslation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.Interpolating.Geometry.Twist2d;
import frc.lib.AccelerationIntegrator;
import frc.lib.VisionOutput;
import frc.lib.Interpolating.InterpolatingDouble;
import frc.lib.Interpolating.InterpolatingTreeMap;
import frc.lib.Interpolating.Interpolable;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants;
import frc.robot.Subsystems.Vision.Vision;
import org.opencv.video.KalmanFilter;

import com.ctre.phoenix6.Timestamp;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Subsystems.CommandSwerveDrivetrain.Drivetrain;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;


public class RobotState { //will estimate pose with odometry and correct drift with vision
    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    private static AccelerationIntegrator accelIntegrator = new AccelerationIntegrator();
    
    Drivetrain drivetrain;
    Pigeon2 pigeon; //getting the already constructed pigeon in swerve

    private InterpolatingTreeMap<InterpolatingDouble, IPose2d> odometryPoses;
	private InterpolatingTreeMap<InterpolatingDouble, ITranslation2d> filteredPoses;
    private ExtendedKalmanFilter<N2, N2, N2> EKF;

    private static final double dt = 0.002;
    private static final int observationSize = 50; //how many poses we keep our tree

    // TODO get more accurate values based on our pigeon std and odo std
    private final static Matrix<N2, N1> stateStdDevs = VecBuilder.fill(0.05,0.05); // obtained from noise when sensor is at rest
    private final static Matrix<N2, N1> measurementStdDevs = VecBuilder.fill(0.02,0.02); // idk how to find this but ill figure it out 

	private Optional<ITranslation2d> initialFieldToOdo = Optional.empty(); //TODO make sure this gets filled (by auto or smth)
    private Optional<EstimatedRobotPose> prevVisionPose;

    private Twist2d robotVelocity;

	private boolean inAuto = false; //need to configure with auto but we dont have an auto yet

    public RobotState() {
        drivetrain = Drivetrain.getInstance();
        pigeon = drivetrain.getPigeon2();
        initKalman();
        reset(0, IPose2d.identity()); //init
    }

    public synchronized void visionUpdate(VisionOutput updatePose) {
        if (prevVisionPose.isEmpty() || initialFieldToOdo.isEmpty()) { //first update
            double timestamp = updatePose.timestampSeconds;

            //merge odom and vision
            ITranslation2d visionTranslation = updatePose.getInterpolatableTransform2d();
            ITranslation2d proximateOdoTranslation = new ITranslation2d(odometryPoses.getInterpolated(new InterpolatingDouble(timestamp)));
            ITranslation2d mergedPose = visionTranslation.weightedAverageBy(proximateOdoTranslation, 0.80); //trust vision more, should tune
            filteredPoses.put(new InterpolatingDouble(timestamp),  mergedPose);
            
            //update kalman
            EKF.setXhat(0, mergedPose.getX());
            EKF.setXhat(1, mergedPose.getY());
            
            //set our initial pose from first update
            initialFieldToOdo = Optional.of(filteredPoses.lastEntry().getValue());
            prevVisionPose = Optional.ofNullable(updatePose); 
        } else { //after first update
            double timestamp = updatePose.timestampSeconds;

            //obtain odometry from interpolation and use the prev pose for the kalman
            ITranslation2d visionTranslation = updatePose.getInterpolatableTransform2d();
            ITranslation2d proximateOdoTranslation = new ITranslation2d(odometryPoses.getInterpolated(new InterpolatingDouble(timestamp)));
            ITranslation2d mergedPose = visionTranslation.weightedAverageBy(proximateOdoTranslation, 0.80); //trust vision more, should tune
            prevVisionPose = Optional.ofNullable(updatePose);

            //calculate std of vision estimate for EKF
            Vector<N2> stdevs = VecBuilder.fill(Math.pow(updatePose.getStandardDeviation(), 1), Math.pow(updatePose.getStandardDeviation(), 1));
					EKF.correct(
							VecBuilder.fill(0.0, 0.0),
							VecBuilder.fill(
									mergedPose.getX(),
									mergedPose.getY()),
							StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), stdevs));
					filteredPoses.put(
							new InterpolatingDouble(timestamp),
							new ITranslation2d(EKF.getXhat(0), EKF.getXhat(1)));
        }
    }

    //Dont need velocity values from odom bc our pigeon is more accurate than slipping wheel encoders
    public synchronized void odometryUpdate(Pose2d pose, double timestamp) {
        odometryPoses.put(new InterpolatingDouble(timestamp), new IPose2d(pose.getX(),pose.getY(), pose.getRotation()));

        // propagate the EKF (with no inputs)
		EKF.predict(VecBuilder.fill(0.0, 0.0), dt);
    }


    public void initKalman() {
        EKF = new ExtendedKalmanFilter<>(Nat.N2(), Nat.N2(), Nat.N2(),
        (x,u) -> u, // return input as the output (f)
        (x,u) -> x, // return states as the output (h)
        stateStdDevs, measurementStdDevs, dt);

        // states - A Nat representing the number of states.
        // outputs - A Nat representing the number of outputs.
        // f - A vector-valued function of x and u that returns the derivative of the state vector.
        // h - A vector-valued function of x and u that returns the measurement vector.
        // stateStdDevs - Standard deviations of model states.
        // measurementStdDevs - Standard deviations of measurements.
        // nominalDtSeconds - Nominal discretization timestep.

        /*  ExtendedKalmanFilter​(Nat<States> states, Nat<Inputs> inputs, Nat<Outputs> outputs,
        BiFunction<Matrix<States,​N1>,​Matrix<Inputs,​N1>,​Matrix<States,​N1>> f,
        BiFunction<Matrix<States,​N1>,​Matrix<Inputs,​N1>,​Matrix<Outputs,​N1>> h,
        Matrix<States,​N1> stateStdDevs, Matrix<Outputs,​N1> measurementStdDevs,
        double dtSeconds)
         */
        }


        public void reset(double time, IPose2d initial_Pose2d) { //init the robot state
            odometryPoses = new InterpolatingTreeMap<>(observationSize);
            odometryPoses.put(new InterpolatingDouble(time), initial_Pose2d);
            filteredPoses = new InterpolatingTreeMap<>(observationSize);
            filteredPoses.put(new InterpolatingDouble(time), getInitialFieldToOdom());
            prevVisionPose = Optional.empty();
        }


        public synchronized ITranslation2d getInitialFieldToOdom() {
            if (initialFieldToOdo.isEmpty()) return ITranslation2d.identity();
            return initialFieldToOdo.get();
        }


        public void updateAccel() {
            double[] newAccel = rawRobotAcceleration();
            robotVelocity = accelIntegrator.update(newAccel[0], newAccel[1], newAccel[2]);
        }

        /**
         * Gets odometry pose from history. Linearly interpolates between gaps.
         *
         * @param timestamp Timestamp to loop up.
         * @return Odometry relative robot pose at timestamp.
         */
        public synchronized IPose2d getOdomToVehicle(double timestamp) {
            return odometryPoses.getInterpolated(new InterpolatingDouble(timestamp));
        }

        // /**
        //  * Gets interpolated odometry pose using predicted robot velocity from latest
        //  * odometry update.
        //  *
        //  * @param lookahead_time Scalar for predicted velocity.
        //  * @return Predcited odometry pose at lookahead time.
        //  */
        // public synchronized Pose2d getPredictedOdomToVehicle(double lookahead_time) {
		//     return getLatestOdomToVehicle()
		// 		.getValue()
		// 		.transformBy(Pose2d.exp(vehicle_velocity_predicted.scaled(lookahead_time)));
	    // } TODO lookahead for auto

        public synchronized ITranslation2d getLatestFilteredPose() {
		    return getFieldToOdom(filteredPoses.lastKey().value);
	    }

        /**
         * Gets odometry error translation at timestamp. Linearly interpolates between gaps.
         * @param timestamp Timestamp to look up.
         * @return Odometry error at timestamp.
         */
        public synchronized ITranslation2d getFieldToOdom(double timestamp) {
            if (filteredPoses.isEmpty()) return ITranslation2d.identity();
            return filteredPoses.getInterpolated(new InterpolatingDouble(timestamp));
        }

        /**
         * Gets velocity from integrated acceleration from Pigeon2
         *
         * @return velocity
         */
        public synchronized double robotVelocityMagnitude() {
            return Math.signum(Math.atan2(robotVelocity.getX(), robotVelocity.getY())) * Math.hypot(robotVelocity.getX(), robotVelocity.getY());
        }

        public synchronized double robotYaw() {
            return pigeon.getYaw().getValue();
        }

        public synchronized double[] robotAngularVelocities(){
            double angularX = pigeon.getAngularVelocityXDevice().getValue();
            double angularY = pigeon.getAngularVelocityYDevice().getValue();

            double timestamp = pigeon.getAngularVelocityXDevice().getTimestamp().getTime();
            
            return new double[] {angularX, angularY, timestamp};
        }

        public synchronized double[] robotAngularVelocityMagnitude(){
            double angularX = pigeon.getAngularVelocityXDevice().getValue();
            double angularY = pigeon.getAngularVelocityYDevice().getValue();

            double timestamp = pigeon.getAngularVelocityXDevice().getTimestamp().getTime();
            
            return new double[] {(Math.signum(Math.atan2(angularY, angularX)) * Math.hypot(angularX, angularY)), timestamp};
        }

    
        public synchronized double[] rawRobotAcceleration() {
            double accelerationX = pigeon.getAccelerationX().getValue() - pigeon.getGravityVectorX().getValue();
            double accelerationY = pigeon.getAccelerationY().getValue() - pigeon.getGravityVectorY().getValue();
            
            double timestamp = pigeon.getAccelerationX().getTimestamp().getTime();

            return new double[] {accelerationX, accelerationY, timestamp};
        }

        


        // -----------------------------------------------------------
                // BiFunction<Matrix<N3, N1>, Matrix<N2, N1>, Matrix<N3, N1>> f = (state, input) -> {
        //     double x = state.get(0, 0);
        //     double y = state.get(1, 0);
        //     double velx = state.get(2, 0);
        //     double vely = state.get(3, 0);
        
        //     double accelx = input.get(0, 0);
        //     double accely = input.get(1, 0);

        //     return VecBuilder.fill(
        //         x + velx * dt,                // New x position
        //         y + vely * dt,                // New y position
        //         vely + accely * dt                // New y velocity
        //         );
        // };

        // BiFunction<Matrix<N3, N1>, Matrix<N2, N1>, Matrix<N2, N1>> h = (stateEstimate, state) -> {
        //     double x = stateEstimate.get(0, 0);
        //     double y = stateEstimate.get(1, 0);
        //     double velx = stateEstimate.get(2, 0);
        //     double vely = stateEstimate.get(3, 0);

        //     return VecBuilder.fill(
        //         x,         //  x position
        //         y,         //  y position
        //         Math.hypot(velx, vely) // velocity magnitude
        //         );
        // };

        // --------- if we ever need a more complicated filter, you can implement it with the code above ------ 
 }