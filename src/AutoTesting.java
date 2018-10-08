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
    private static SuperFunAutoUtility autoUtility;

    public static void main(String[] args) {
        EasyDrawing.beginDrawing();
        //EasyDrawing.addRectangle(new Rectangle(0, 0, 100, 100));
        //TODO: Create EasyDrawing plot pose method to visualize easy

        autoUtility = new SuperFunAutoUtility();
        AutoTesting test = new AutoTesting();
        test.testAutoRoutine();
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * 2048.0 / 10.0;
    }

    //Pseudo Wrapper Method - Interface to hardware.
    public static void setOutput(DriveSignal signal, DriveSignal feedforward, double left_accel, double right_accel) {
        double left_demand = radiansPerSecondToTicksPer100ms(signal.getLeft());
        double right_demand = radiansPerSecondToTicksPer100ms(signal.getRight());
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
    }

    public void testAutoRoutine() {
        /*
        autoUtility.setTrajectory(new TrajectoryIterator<>(new TimedView<>(autoUtility.generateTrajectory
                (false, Arrays.asList(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.00)), new Pose2d(new Translation2d(4.25, -0.25), Rotation2d.fromDegrees(10)), new Pose2d(new Translation2d(6, 4.25), Rotation2d.fromDegrees(85)), new Pose2d(new Translation2d(6.75, 5), Rotation2d.fromDegrees(-5))),
                        Arrays.asList(new CentripetalAccelerationConstraint(120.0)),
                        60.0, 120.0, 10.0))));
                        */
        autoUtility.setTrajectory(new TrajectoryIterator<>(new TimedView<>(autoUtility.generateTrajectory
                (false, Arrays.asList(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.00)), new Pose2d(new Translation2d(100, 100), Rotation2d.fromDegrees(0))),
                        Arrays.asList(new CentripetalAccelerationConstraint(120.0)),
                        120.0, 120.0, 10.0))));

        double t = 0.0;
        Pose2d pose = autoUtility.setpoint().state().getPose();
        while (!autoUtility.isDone()) {
            EasyDrawing.addRobotPose(pose.getTranslation().x(), pose.getTranslation().y());
            updatePathFollower(t, pose);
            pose = autoUtility.mSetpoint.state().getPose(); //Feed current pose back for testing purposes so that the robot "moves"... use the kinematics!
            t += 0.01;
        }
    }

    //Pseudo Wrapper Method
    public void updatePathFollower(double now, Pose2d currentRobotPose) {
        SuperFunAutoUtility.Output output = autoUtility.update(now, currentRobotPose);

        Pose2d error = autoUtility.error();
        TimedState<Pose2dWithCurvature> path_setpoint = autoUtility.setpoint();

        double left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
        double right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;

        //System.out.println(currentRobotPose);

        System.out.println(output.left_feedforward_voltage + " " + output.right_feedforward_voltage);

        setOutput(new DriveSignal(output.left_velocity, output.right_velocity),
                new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0), left_accel, right_accel);
    }
}
