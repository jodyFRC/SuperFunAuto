package lib;

import lib.geometry.Rotation2d;
import lib.geometry.Twist2d;

public class RobotStateEstimator {
    static RobotStateEstimator instance_ = new RobotStateEstimator();
    private RobotState robot_state_ = RobotState.getInstance();
    private double left_encoder_prev_distance_ = 0.0;
    private double right_encoder_prev_distance_ = 0.0;

    RobotStateEstimator() {
    }

    public static RobotStateEstimator getInstance() {
        return instance_;
    }

    public void onStart(double timestamp, double left_encoder_distance_, double right_encoder_distance_) {
        left_encoder_prev_distance_ = left_encoder_distance_;
        right_encoder_prev_distance_ = right_encoder_distance_;

    }

    public void onLoop(double timestamp, double left_encoder_distance_, double right_encoder_distance_, double left_velocity_linear, double right_velocity_linear, double heading) {
        final double left_distance = left_encoder_distance_;
        final double right_distance = right_encoder_distance_;
        final double delta_left = left_distance - left_encoder_prev_distance_;
        final double delta_right = right_distance - right_encoder_prev_distance_;
        final Rotation2d gyro_angle = Rotation2d.fromDegrees(heading);
        final Twist2d odometry_velocity = robot_state_.generateOdometryFromSensors(
                delta_left, delta_right, gyro_angle);
        final Twist2d predicted_velocity = Kinematics.forwardKinematics(left_velocity_linear,
                right_velocity_linear);
        robot_state_.addObservations(timestamp, odometry_velocity,
                predicted_velocity);
        left_encoder_prev_distance_ = left_distance;
        right_encoder_prev_distance_ = right_distance;
    }
}
