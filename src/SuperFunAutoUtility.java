import lib.geometry.*;
import lib.physics.DCMotorTransmission;
import lib.physics.DifferentialDrive;
import lib.trajectory.*;
import lib.trajectory.timing.DifferentialDriveDynamicsConstraint;
import lib.trajectory.timing.TimedState;
import lib.trajectory.timing.TimingConstraint;
import lib.trajectory.timing.TimingUtil;
import lib.util.Units;
import lib.util.Util;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

public class SuperFunAutoUtility {

    DifferentialDrive mModel;
    TrajectoryIterator<TimedState<Pose2dWithCurvature>> mCurrentTrajectory;
    boolean mIsReversed = false;
    double mLastTime = Double.POSITIVE_INFINITY;
    public TimedState<Pose2dWithCurvature> mSetpoint = new TimedState<>(Pose2dWithCurvature.identity());
    Pose2d mError = Pose2d.identity();
    Output mOutput = new Output();

    public static final double kPathKX = 4.0;  // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4;  // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0;  // inches

    public static final double kRobotLinearInertia = 60.0;  // kg TODO tune
    public static final double kRobotAngularInertia = 10.0;  // kg m^2 TODO tune
    public static final double kRobotAngularDrag = 12.0;  // N*m / (rad/sec) TODO tune
    public static final double kDriveVIntercept = 1.055;  // V
    public static final double kDriveKv = 0.135;  // V per rad/s
    public static final double kDriveKa = 0.012;  // V per rad/s^2

    public static final double kDriveWheelTrackWidthInches = 25.54;
    public static final double kDriveWheelDiameterInches = 3.92820959548 * 0.99;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 1.0;  // Tune me!

    private static final double kMaxDx = 2.0;
    private static final double kMaxDy = 0.25;
    private static final double kMaxDTheta = Math.toRadians(5.0);

    double mDt = 0.0;


    public SuperFunAutoUtility() {
        final DCMotorTransmission transmission = new DCMotorTransmission(
                1.0 / kDriveKv,
                Units.inches_to_meters(kDriveWheelRadiusInches) * Units.inches_to_meters(kDriveWheelRadiusInches) * kRobotLinearInertia / (2.0 * kDriveKa),
                kDriveVIntercept);
        mModel = new DifferentialDrive(
                kRobotLinearInertia,
                kRobotAngularInertia,
                kRobotAngularDrag,
                Units.inches_to_meters(kDriveWheelDiameterInches / 2.0),
                Units.inches_to_meters(kDriveWheelTrackWidthInches / 2.0 * kTrackScrubFactor),
                transmission, transmission
        );
    }

    public String toCSV() {
        DecimalFormat fmt = new DecimalFormat("#0.000");
        return fmt.format(mOutput.left_velocity) + "," + fmt.format(mOutput.right_velocity) + "," + fmt.format
                (mOutput.left_feedforward_voltage) + "," + fmt.format(mOutput.right_feedforward_voltage) + "," +
                mSetpoint.toCSV();
    }

    public boolean isDone() {
        return mCurrentTrajectory != null && mCurrentTrajectory.isDone();
    }

    public Pose2d error() {
        return mError;
    }

    public TimedState<Pose2dWithCurvature> setpoint() {
        return mSetpoint;
    }

    public void setTrajectory(final TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        mCurrentTrajectory = trajectory;
        mSetpoint = trajectory.getState();
        for (int i = 0; i < trajectory.trajectory().length(); ++i) {
            if (trajectory.trajectory().getState(i).velocity() > Util.kEpsilon) {
                mIsReversed = false;
                break;
            } else if (trajectory.trajectory().getState(i).velocity() < -Util.kEpsilon) {
                mIsReversed = true;
                break;
            }
        }
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return generateTrajectory(reversed, waypoints, constraints, 0.0, 0.0, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,
            double end_vel,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        List<Pose2d> waypoints_maybe_flipped = waypoints;
        final Pose2d flip = Pose2d.fromRotation(new Rotation2d(-1, 0, false));
        // TODO re-architect the spline generator to support reverse.
        if (reversed) {
            waypoints_maybe_flipped = new ArrayList<>(waypoints.size());
            for (int i = 0; i < waypoints.size(); ++i) {
                waypoints_maybe_flipped.add(waypoints.get(i).transformBy(flip));
            }
        }

        // Create a trajectory from splines.
        Trajectory<Pose2dWithCurvature> trajectory = TrajectoryUtil.trajectoryFromSplineWaypoints(
                waypoints_maybe_flipped, kMaxDx, kMaxDy, kMaxDTheta);

        if (reversed) {
            List<Pose2dWithCurvature> flipped = new ArrayList<>(trajectory.length());
            for (int i = 0; i < trajectory.length(); ++i) {
                flipped.add(new Pose2dWithCurvature(trajectory.getState(i).getPose().transformBy(flip), -trajectory
                        .getState(i).getCurvature(), trajectory.getState(i).getDCurvatureDs()));
            }
            trajectory = new Trajectory<>(flipped);
        }
        // Create the constraint that the robot must be able to traverse the trajectory without ever applying more
        // than the specified voltage.
        final DifferentialDriveDynamicsConstraint<Pose2dWithCurvature> drive_constraints = new
                DifferentialDriveDynamicsConstraint<>(mModel, max_voltage);
        List<TimingConstraint<Pose2dWithCurvature>> all_constraints = new ArrayList<>();
        all_constraints.add(drive_constraints);
        if (constraints != null) {
            all_constraints.addAll(constraints);
        }
        // Generate the timed trajectory.
        Trajectory<TimedState<Pose2dWithCurvature>> timed_trajectory = TimingUtil.timeParameterizeTrajectory
                (reversed, new
                        DistanceView<>(trajectory), kMaxDx, all_constraints, start_vel, end_vel, max_vel, max_accel);
        return timed_trajectory;
    }

    public Output update(double timestamp, Pose2d current_state) {
        if (mCurrentTrajectory == null) return new Output();

        if (mCurrentTrajectory.getProgress() == 0.0 && !Double.isFinite(mLastTime)) {
            mLastTime = timestamp;
        }

        mDt = timestamp - mLastTime;
        mLastTime = timestamp;
        TrajectorySamplePoint<TimedState<Pose2dWithCurvature>> sample_point = mCurrentTrajectory.advance(mDt);
        mSetpoint = sample_point.state();

        if (!mCurrentTrajectory.isDone()) {
            // Generate feedforward voltages.
            final double velocity_m = Units.inches_to_meters(mSetpoint.velocity());
            final double curvature_m = Units.meters_to_inches(mSetpoint.state().getCurvature());
            final double dcurvature_ds_m = Units.meters_to_inches(Units.meters_to_inches(mSetpoint.state()
                    .getDCurvatureDs()));
            final double acceleration_m = Units.inches_to_meters(mSetpoint.acceleration());
            final DifferentialDrive.DriveDynamics dynamics = mModel.solveInverseDynamics(
                    new DifferentialDrive.ChassisState(velocity_m, velocity_m * curvature_m),
                    new DifferentialDrive.ChassisState(acceleration_m,
                            acceleration_m * curvature_m + velocity_m * velocity_m * dcurvature_ds_m));
            mError = current_state.inverse().transformBy(mSetpoint.state().getPose());
            mOutput = updatePurePursuit(dynamics, current_state);
        }

        return mOutput;
    }

    protected Output updatePurePursuit(DifferentialDrive.DriveDynamics dynamics, Pose2d current_state) {
        double lookahead_time = kPathLookaheadTime;
        final double kLookaheadSearchDt = 0.01;
        TimedState<Pose2dWithCurvature> lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
        double actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
        while (actual_lookahead_distance < kPathMinLookaheadDistance &&
                mCurrentTrajectory.getRemainingProgress() > lookahead_time) {
            lookahead_time += kLookaheadSearchDt;
            lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
            actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
        }
        if (actual_lookahead_distance < kPathMinLookaheadDistance) {
            lookahead_state = new TimedState<>(new Pose2dWithCurvature(lookahead_state.state()
                    .getPose().transformBy(Pose2d.fromTranslation(new Translation2d(
                            (mIsReversed ? -1.0 : 1.0) * (kPathMinLookaheadDistance -
                                    actual_lookahead_distance), 0.0))), 0.0), lookahead_state.t()
                    , lookahead_state.velocity(), lookahead_state.acceleration());
        }

        DifferentialDrive.ChassisState adjusted_velocity = new DifferentialDrive.ChassisState();
        // Feedback on longitudinal error (distance).
        adjusted_velocity.linear = dynamics.chassis_velocity.linear + kPathKX * Units.inches_to_meters
                (mError.getTranslation().x());

        // Use pure pursuit to peek ahead along the trajectory and generate a new curvature.
        final PurePursuitController.Arc<Pose2dWithCurvature> arc = new PurePursuitController.Arc<>(current_state,
                lookahead_state.state());

        double curvature = 1.0 / Units.inches_to_meters(arc.radius);
        if (Double.isInfinite(curvature)) {
            adjusted_velocity.linear = 0.0;
            adjusted_velocity.angular = dynamics.chassis_velocity.angular;
        } else {
            adjusted_velocity.angular = curvature * dynamics.chassis_velocity.linear;
        }

        dynamics.chassis_velocity = adjusted_velocity;
        dynamics.wheel_velocity = mModel.solveInverseKinematics(adjusted_velocity);
        return new Output(dynamics.wheel_velocity.left, dynamics.wheel_velocity.right, dynamics.wheel_acceleration
                .left, dynamics.wheel_acceleration.right, dynamics.voltage.left, dynamics.voltage.right);
    }

    public static class Output {
        public Output() {
        }

        public Output(double left_velocity, double right_velocity, double left_accel, double right_accel,
                      double left_feedforward_voltage, double
                              right_feedforward_voltage) {
            this.left_velocity = left_velocity;
            this.right_velocity = right_velocity;
            this.left_accel = left_accel;
            this.right_accel = right_accel;
            this.left_feedforward_voltage = left_feedforward_voltage;
            this.right_feedforward_voltage = right_feedforward_voltage;
        }

        public double left_velocity;  // rad/s
        public double right_velocity;  // rad/s

        public double left_accel;  // rad/s^2
        public double right_accel;  // rad/s^2

        public double left_feedforward_voltage;
        public double right_feedforward_voltage;

        public void flip() {
            double tmp_left_velocity = left_velocity;
            left_velocity = -right_velocity;
            right_velocity = -tmp_left_velocity;

            double tmp_left_accel = left_accel;
            left_accel = -right_accel;
            right_accel = -tmp_left_accel;

            double tmp_left_feedforward = left_feedforward_voltage;
            left_feedforward_voltage = -right_feedforward_voltage;
            right_feedforward_voltage = -tmp_left_feedforward;
        }
    }

}