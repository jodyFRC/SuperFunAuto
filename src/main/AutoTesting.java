package main;

import lib.RobotState;
import lib.RobotStateEstimator;
import lib.geometry.Pose2d;
import lib.geometry.Pose2dWithCurvature;
import lib.geometry.Rotation2d;
import lib.geometry.Translation2d;
import lib.trajectory.TimedView;
import lib.trajectory.TrajectoryIterator;
import lib.trajectory.timing.CentripetalAccelerationConstraint;
import lib.trajectory.timing.TimedState;
import lib.util.DriveSignal;

import java.awt.*;
import java.util.Arrays;

/**
 * Created by Jody on 10/2/2018.
 */
public class AutoTesting extends Canvas {

    public static final double kDriveLowGearVelocityKp = 0.9;
    public static final double kDriveLowGearVelocityKi = 0.0;
    public static final double kDriveLowGearVelocityKd = 10.0;
    public static final double kDriveLowGearVelocityKf = 0.0;
    public static final int kDriveLowGearVelocityIZone = 0;
    public static final double kDriveVoltageRampRate = 0.0;
    static DrivetrainControl drivetrainControl = new DrivetrainControl();
    private static SuperFunAutoUtility autoUtility;
    double dt = 0.005;
    double left_encoder_dist = 0;
    double right_encoder_dist = 0;
    double gyro_angle = 0;
    RobotStateEstimator stateEstimator = RobotStateEstimator.getInstance();

    public static void main(String[] args) {
        main.EasyDrawing.beginDrawing();
        //TODO: Create main.EasyDrawing plot pose method to visualize easy

        autoUtility = new SuperFunAutoUtility();
        AutoTesting test = new AutoTesting();
        test.testAutoRoutine();
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * 2048.0 / 10.0;
    }

    //Pseudo Wrapper Method - Interface to hardware.
    public static void setOutput(DriveSignal signal, DriveSignal feedforward, double left_accel, double right_accel) {
        double left_feedforward = feedforward.getLeft() + (kDriveLowGearVelocityKd * left_accel / 1024.0);
        double right_feedforward = feedforward.getRight() + (kDriveLowGearVelocityKd * right_accel / 1024.0);


        //System.out.println("Left Motor Velocity RPS Demand:" + signal.getLeft()/(2 * Math.PI));
        //System.out.println("Right Motor Velocity RPS Demand:" + signal.getRight()/(2 * Math.PI));
        //System.out.println("Left Motor FF Normalized Output:" + left_feedforward);
        //System.out.println("Right Motor FF Normalized Output:" + right_feedforward);
        //System.out.println("----------------------------------");


        /*  Real Hardware Impl.
                    mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.left_feedforward + Constants.kDriveLowGearVelocityKd * mPeriodicIO.left_accel / 1023.0);
                    mRightMaster.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.right_feedforward + Constants.kDriveLowGearVelocityKd * mPeriodicIO.right_accel / 1023.0);
         */

        drivetrainControl.setDesired(signal.getLeft(), signal.getRight(), left_feedforward, right_feedforward);
        drivetrainControl.update(signal.getLeft() + (Math.random() * 0.1 - 0.05), signal.getRight() + (Math.random() * 0.1 - 0.05)); //TODO: For testing, we feed back our desired as actual with some (error)
    }

    public void testAutoRoutine() {
        autoUtility.setTrajectory(new TrajectoryIterator<>(new TimedView<>(autoUtility.generateTrajectory
                (false, Arrays.asList(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0.00)), new Pose2d(new Translation2d(45, 1.5), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(60, 40), Rotation2d.fromDegrees(90)), new Pose2d(new Translation2d(65, 50), Rotation2d.fromDegrees(-10))),
                        Arrays.asList(new CentripetalAccelerationConstraint(60.0)),
                        120.0, 120.0, 10.0))));

        double t = 0.0;
        Pose2d pose;

        RobotState.getInstance().reset(0.0, new Pose2d(new Translation2d(1, 1), Rotation2d.fromDegrees(0.00))); //Initial Robot State (we can add error by having it start initially at a point that the trajectory doesn't start at!)

        while (!autoUtility.isDone()) {
            //pose = autoUtility.mSetpoint.state().getPose(); //Feed current pose back for testing purposes so that the robot "moves"... use the kinematics!
            pose = RobotState.getInstance().getFieldToVehicle(t); //Test by feeding back calculated physical encoder values via integration and then applying kinematics

            updatePathFollower(t, pose);
            main.EasyDrawing.addRobotPose(pose.getTranslation().x(), pose.getTranslation().y(), Color.BLUE);

            try {
                Thread.sleep(20);
            } catch (Exception e) {
                e.printStackTrace();
            }

            t += dt;
        }
        System.out.println("Time: " + t);
    }

    public double rotationToLinear(double a) {
        return a * SuperFunAutoUtility.kDriveWheelRadiusInches;
    }

    //Pseudo Wrapper Method
    public void updatePathFollower(double now, Pose2d currentRobotPose) {
        SuperFunAutoUtility.Output output = autoUtility.update(now, currentRobotPose);

        //dirty integration . . . we are assuming the robot can match wheel velo with desired velos
        //also assume robot can get the desired angular velocity it wants instantly

        //---------------------------------------------
        //On Real Robot, these need to be sensor driven values
        left_encoder_dist = left_encoder_dist + ((output.left_velocity * 1) * dt);  //radians
        right_encoder_dist = right_encoder_dist + ((output.right_velocity * 1) * dt);
        gyro_angle = gyro_angle + (output.angular_velocity);
        //---------------------------------------------

        Pose2d error = autoUtility.error();
        TimedState<Pose2dWithCurvature> path_setpoint = autoUtility.setpoint();
        main.EasyDrawing.addRobotPose(path_setpoint.state().getTranslation().x(), path_setpoint.state().getTranslation().y(), Color.RED);

        double x_a = currentRobotPose.getTranslation().x() + (8.3 * Math.cos(Math.toRadians(currentRobotPose.getRotation().getDegrees())));
        double y_a = currentRobotPose.getTranslation().y() + (8.3 * Math.sin(Math.toRadians(currentRobotPose.getRotation().getDegrees())));

        double x_b = currentRobotPose.getTranslation().x() + (6.5 * Math.cos(Math.toRadians(currentRobotPose.getRotation().getDegrees() + 90)));
        double y_b = currentRobotPose.getTranslation().y() + (6.5 * Math.sin(Math.toRadians(currentRobotPose.getRotation().getDegrees() + 90)));

        double x_c = x_a + (6.5 * Math.cos(Math.toRadians(currentRobotPose.getRotation().getDegrees() + 90)));
        double y_c = y_a + (6.5 * Math.sin(Math.toRadians(currentRobotPose.getRotation().getDegrees() + 90)));

        double dist_front_back = Math.sqrt(Math.pow((x_a - currentRobotPose.getTranslation().x()), 2) + Math.pow((y_a - currentRobotPose.getTranslation().y()), 2));

        main.EasyDrawing.addRobotPose(x_a, y_a, Color.GREEN);
        main.EasyDrawing.addRobotPose(x_b, y_b, Color.PINK);
        main.EasyDrawing.addRobotPose(x_c, y_c, Color.ORANGE);

        double left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
        double right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;

        //we don't care about these...
        stateEstimator.onLoop(now, rotationToLinear(left_encoder_dist), rotationToLinear(right_encoder_dist), 0, 0, gyro_angle);
        //throw setpoint rotation back and just treat as gyro for now

        System.out.println("error: " + error.getPose().toString());
        System.out.println("wheel velos: " + rotationToLinear(output.left_velocity) + " " + rotationToLinear(output.right_velocity));
        EasyDrawing.debug_info = rotationToLinear(output.left_velocity) + " " + rotationToLinear(output.right_velocity) + "          " + currentRobotPose.getRotation().getDegrees() + "        " + dist_front_back;
        System.out.println("-----");

        setOutput(new DriveSignal(output.left_velocity, output.right_velocity),
                new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0), left_accel, right_accel);
    }
}
