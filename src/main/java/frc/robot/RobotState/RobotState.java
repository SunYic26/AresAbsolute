package frc.robot.RobotState;

import java.io.ObjectInputStream.GetField;
import java.util.List;
import java.util.Optional;
import java.util.TreeMap;
import java.util.function.BiFunction;
import java.util.function.Supplier;

import javax.swing.text.html.Option;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.interpolation.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.UnitBuilder;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
import edu.wpi.first.math.estimator.PoseEstimator;
import frc.lib.Interpolating.Geometry.IPose2d;
import frc.lib.Interpolating.Geometry.ITranslation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import frc.lib.Interpolating.Geometry.ITwist2d;
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

import org.ejml.equation.MatrixConstructor;
import org.opencv.video.KalmanFilter;

import com.ctre.phoenix6.Timestamp;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Subsystems.CommandSwerveDrivetrain.Drivetrain;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;

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
    private InterpolatingTreeMap<InterpolatingDouble, ITwist2d> robotVelocities;

    private UnscentedKalmanFilter<N2, N2, N2> UKF;

    private static final double dt = 0.020;
    private static final int observationSize = 50; //how many poses we keep our tree

	private Optional<ITranslation2d> initialFieldToOdo = Optional.empty();
    private Optional<EstimatedRobotPose> prevVisionPose;
    private Optional<Double> prevOdomTimestamp = Optional.empty();

	private boolean inAuto = false; //need to configure with auto but we dont have an auto yet (lol)

    public RobotState() {
        drivetrain = Drivetrain.getInstance();
        pigeon = drivetrain.getPigeon2();
        initKalman();
        reset(0.02, IPose2d.identity(), ITwist2d.identity()); //init
    }

    public synchronized void visionUpdate(VisionOutput updatePose) {
        if (prevVisionPose.isEmpty() || initialFieldToOdo.isEmpty()) { //first update
            double timestamp = updatePose.timestampSeconds;

            //merge odom and vision for first run
            ITranslation2d visionTranslation = updatePose.getInterpolatableTransform2d();
            ITranslation2d proximateOdoTranslation = new ITranslation2d(odometryPoses.getInterpolated(new InterpolatingDouble(timestamp)));
            ITranslation2d mergedPose = visionTranslation.weightedAverageBy(proximateOdoTranslation, 0.70); //trust vision more, should tune
            filteredPoses.put(new InterpolatingDouble(timestamp),  mergedPose);
            
            //update kalman
            UKF.setXhat(0, mergedPose.getX());
            UKF.setXhat(1, mergedPose.getY());
            
            //set our initial pose from first update
            initialFieldToOdo = Optional.of(filteredPoses.lastEntry().getValue());
            prevVisionPose = Optional.ofNullable(updatePose); 

        } else { //after first update

            double timestamp = updatePose.timestampSeconds;

            prevVisionPose = Optional.ofNullable(updatePose);

            updateAccel();

            ITwist2d robotVelocity = getInterpolatedValue(robotVelocities, timestamp, ITwist2d.identity());

            UKF.predict(VecBuilder.fill(robotVelocity.getX(), robotVelocity.getY()), timestamp);

            //calculate std of vision estimate for UKF
            Vector<N2> stdevs = VecBuilder.fill(Math.pow(updatePose.getStandardDeviation(), 1), Math.pow(updatePose.getStandardDeviation(), 1));
					UKF.correct(
							VecBuilder.fill(robotVelocity.getX(), robotVelocity.getY()),
							VecBuilder.fill(
									updatePose.estimatedPose.getX(),
									updatePose.estimatedPose.getY()),
							StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), stdevs));
					filteredPoses.put(
							new InterpolatingDouble(timestamp),
							new ITranslation2d(UKF.getXhat(0), UKF.getXhat(1)));
        }
    }

    int k = 0;

    //Dont need velocity values from odom bc our pigeon is more accurate than slipping wheel encoders
    public synchronized void odometryUpdate(Pose2d pose, double[] wheelVelocity, double timestamp) {

        updateAccel(wheelVelocity);
        ITwist2d robotVelocity = getLatestRobotVelocity();

        if(Math.abs(robotVelocityMagnitude()) > 0) { //manually increase P
             Matrix<N2,N2> P = UKF.getP();
             P.set(0, 0, P.get(0, 0) + 0.002);
             P.set(1, 1, P.get(1, 1) + 0.002);
             UKF.setP(P);
        }

        if(prevOdomTimestamp.isEmpty()) {
            Matrix<N2, N1> initialState = VecBuilder.fill(pose.getX(), pose.getY());
            UKF.setXhat(initialState);
        } else {
            // robotVelocity = getInterpolatedValue(robotVelocities, prevOdomTimestamp.get(), ITwist2d.identity());

            UKF.predict(VecBuilder.fill(robotVelocity.getX(), robotVelocity.getY()), timestamp);

            UKF.correct(
                VecBuilder.fill(
                    robotVelocity.getX(),
                    robotVelocity.getY()),
                VecBuilder.fill(
                    pose.getX(),
                    pose.getY()));
        }

        odometryPoses.put(new InterpolatingDouble(timestamp), new IPose2d(pose.getX(),pose.getY(), pose.getRotation()));

        prevOdomTimestamp.equals(timestamp);

        System.out.println("P " + UKF.getP().toString());

        SmartDashboard.putNumber("FILT X", UKF.getXhat(0));
        SmartDashboard.putNumber("FILT Y", UKF.getXhat(1));
    }


    public void initKalman() {

        Matrix<N2, N1> stateStdDevsQ = VecBuilder.fill(
            Math.pow(0.0, 1), // Variance in position x
            Math.pow(0.0, 1)  // Variance in position y
        );

        Matrix<N2, N1> measurementStdDevsR = VecBuilder.fill(
            Math.pow(0.003, 1), // Variance in measurement x
            Math.pow(0.003, 1)   // Variance in measurement y
        );

        UKF = new UnscentedKalmanFilter<>(Nat.N2(), Nat.N2(),
        (x,u) -> u,
        (x,u) -> x,
        stateStdDevsQ,
        measurementStdDevsR,
        dt);

        // Set initial state and covariance
        Matrix<N2, N2> initialCovariance = MatBuilder.fill(Nat.N2(), Nat.N2(),
            0.003, 0.0,
            0.0, 0.003
        );

        UKF.setP(initialCovariance); 

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


        public void reset(double time, IPose2d initial_Pose2d, ITwist2d initial_Twist2d) { //init the robot state
            odometryPoses = new InterpolatingTreeMap<>(observationSize);
            odometryPoses.put(new InterpolatingDouble(time), initial_Pose2d);
            filteredPoses = new InterpolatingTreeMap<>(observationSize);
            filteredPoses.put(new InterpolatingDouble(time), getInitialFieldToOdom());
            robotVelocities = new InterpolatingTreeMap<>(observationSize);
            robotVelocities.put(new InterpolatingDouble(time), initial_Twist2d);
            prevVisionPose = Optional.empty();
        }


        public synchronized ITranslation2d getInitialFieldToOdom() {
            if (initialFieldToOdo.isEmpty()) return ITranslation2d.identity();
            return initialFieldToOdo.get();
        }


        public void updateAccel() {
            double[] newAccel = rawRobotAcceleration();
            SmartDashboard.putNumber("raw Accel X", newAccel[0]);
            SmartDashboard.putNumber("raw Accel Y", newAccel[1]);
            robotVelocities.put(new InterpolatingDouble(newAccel[2]), accelIntegrator.update(newAccel));
        }

        public void updateAccel(double[] wheelVelocity) {
            double[] newAccel = rawRobotAcceleration();
            SmartDashboard.putNumber("raw Accel X", newAccel[0]);
            SmartDashboard.putNumber("raw Accel Y", newAccel[1]);
            robotVelocities.put(new InterpolatingDouble(newAccel[2]), accelIntegrator.update(newAccel, wheelVelocity));
        }

        // =======---===[ ⚙ Tree map helpers ]===---========

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

        /**
         * Gets odometry pose from history. Linearly interpolates between gaps.
         *
         * @param timestamp Timestamp to loop up
         * @return Odometry relative robot pose at timestamp
         */
        public synchronized IPose2d getOdometryPose(double timestamp) {
            return odometryPoses.getInterpolated(new InterpolatingDouble(timestamp));
        }

        public synchronized ITranslation2d getLatestFilteredPose() {
		    return getInterpolatedValue(filteredPoses, filteredPoses.lastKey().value, ITranslation2d.identity());
	    }

        public synchronized ITwist2d getLatestRobotVelocity() {
		    return getInterpolatedValue(robotVelocities, robotVelocities.lastKey().value, ITwist2d.identity());
	    }

        /**
         * Gets odometry error translation at timestamp. Linearly interpolates between gaps.
         * @param map Map to look in
         * @param timestamp Timestamp to look up
         * @param identity Identity of the value
         * @return Interpolated value at timestep
         */
        public synchronized <T extends Interpolable<T>> T getInterpolatedValue(InterpolatingTreeMap<InterpolatingDouble, T> map, Double timestamp, T identity) {

        if (map.isEmpty())
            return identity;

        if (timestamp == null) 
            return map.get(map.lastKey());

        // Interpolate for the given timestamp
        return map.getInterpolated(new InterpolatingDouble(timestamp));
        }
    
    
    
        /**
         * Gets odometry error translation at timestamp. Linearly interpolates between gaps.
         * @param timestamp Timestamp to look up
         * @return Twist2d Velocities at timestamp
         */
        public synchronized ITwist2d getVelocityAt(double timestamp) {
            if (robotVelocities.isEmpty()) return ITwist2d.identity();
            return robotVelocities.getInterpolated(new InterpolatingDouble(timestamp));
        }
        
        /**
         * Gets velocity from integrated acceleration from filtered velocities
         *
         * @return double VelocityMagnitude
         */
        public synchronized double robotVelocityMagnitude() {
            ITwist2d robotVelocity = getLatestRobotVelocity();
            return Math.signum(Math.atan2(robotVelocity.getX(), robotVelocity.getY()))
             * Math.hypot(robotVelocity.getX(), robotVelocity.getY());
        }


        //// =======---===[ ⚙ Pigeon2.0  ]===---========

        public synchronized double robotYaw() {
            return pigeon.getYaw().getValue();
        }

        /**
         * Gets the robot angular velocities from the pigeon
         *
         * @return double[] {AngularX, AngularY, Timestamp}
         */
        public synchronized double[] robotAngularVelocities(){
            double angularX = pigeon.getAngularVelocityXDevice().getValue();
            double angularY = pigeon.getAngularVelocityYDevice().getValue();

            double timestamp = pigeon.getAngularVelocityXDevice().getTimestamp().getTime();
            
            return new double[] {angularX, angularY, timestamp};
        }

        /**
         * Gets the robot angular magnitude from the pigeon
         *
         * @return double[] {AngularMagnitude, Timestamp}
         */
        public synchronized double[] robotAngularVelocityMagnitude(){
            double angularX = pigeon.getAngularVelocityXDevice().getValue();
            double angularY = pigeon.getAngularVelocityYDevice().getValue();

            double timestamp = pigeon.getAngularVelocityXDevice().getTimestamp().getTime();
            
            return new double[] {(Math.signum(Math.atan2(angularY, angularX)) * Math.hypot(angularX, angularY)), timestamp};
        }

        /**
         * Gets the robot accelerations from the pigeon
         *
         * @return double[] {AccelerationX, AccelerationY, Timestamp}
         */
        public synchronized double[] rawRobotAcceleration() {
            double accelerationX = (pigeon.getAccelerationX().getValue() - pigeon.getGravityVectorX().getValue()) * 9.80665;
            double accelerationY = (pigeon.getAccelerationY().getValue() - pigeon.getGravityVectorY().getValue()) * 9.80665;
            
            double timestamp = pigeon.getAccelerationX().getTimestamp().getTime();
            SmartDashboard.putNumber("current timestamp", timestamp);
            return new double[] {accelerationX, accelerationY, timestamp};
        }
 }