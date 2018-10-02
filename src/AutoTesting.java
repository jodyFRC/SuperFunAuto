import lib.geometry.Pose2d;
import lib.geometry.Pose2dWithCurvature;
import lib.geometry.Rotation2d;
import lib.geometry.Translation2d;
import lib.trajectory.TimedView;
import lib.trajectory.TrajectoryIterator;
import lib.trajectory.timing.CentripetalAccelerationConstraint;
import lib.trajectory.timing.TimedState;
import lib.util.DriveSignal;

import java.util.Arrays;

/**
 * Created by Jody on 10/2/2018.
 */
public class AutoTesting {

    private static SuperFunAutoUtility autoUtility;

    public static final double kDriveLowGearVelocityKp = 0.9;
    public static final double kDriveLowGearVelocityKi = 0.0;
    public static final double kDriveLowGearVelocityKd = 10.0;
    public static final double kDriveLowGearVelocityKf = 0.0;
    public static final int kDriveLowGearVelocityIZone = 0;
    public static final double kDriveVoltageRampRate = 0.0;

    public static void main(String[] args) {
        autoUtility = new SuperFunAutoUtility();
        AutoTesting test = new AutoTesting();
        //test.testForwardSwerveRight();

        test.testAutoRoutine();
    }

    public void testAutoRoutine() {
        autoUtility.setTrajectory(new TrajectoryIterator<>(new TimedView<>(autoUtility.generateTrajectory
                (false, Arrays.asList(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.identity()),
                        new Pose2d(new Translation2d(120.0, -36.0), Rotation2d.identity()),
                        new Pose2d(new Translation2d(240.0, -36.0), Rotation2d.identity())),
                        Arrays.asList(new CentripetalAccelerationConstraint(120.0)),
                        120.0, 120.0, 10.0))));
        double t = 0.0;
        Pose2d pose = autoUtility.setpoint().state().getPose();
        while (!autoUtility.isDone()) {
            updatePathFollower(t, pose);
            pose = autoUtility.mSetpoint.state().getPose(); //Feed current pose back for testing purposes so that the robot "moves"
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

        System.out.println(error);

        setOutput(new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity), radiansPerSecondToTicksPer100ms(output.right_velocity)),
                new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0), left_accel, right_accel);
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0;
    }

    //Pseudo Wrapper Method - Interface to hardware.
    public static void setOutput(DriveSignal signal, DriveSignal feedforward, double left_accel, double right_accel) {
        double left_demand = signal.getLeft();
        double right_demand = signal.getRight();
        double left_feedforward = feedforward.getLeft() + (kDriveLowGearVelocityKd * left_accel / 1023.0);
        double right_feedforward = feedforward.getRight() + (kDriveLowGearVelocityKd * right_accel / 1023.0);

        System.out.println("Left Motor Demand:" + left_demand);
        System.out.println("Right Motor Demand:" + right_demand);
        System.out.println("Left Motor FF:" + left_feedforward);
        System.out.println("Right Motor FF:" + right_feedforward);
        System.out.println("----------------------------------");


        /*  Real Hardware Impl.
                    mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.left_feedforward + Constants.kDriveLowGearVelocityKd * mPeriodicIO.left_accel / 1023.0);
                    mRightMaster.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.right_feedforward + Constants.kDriveLowGearVelocityKd * mPeriodicIO.right_accel / 1023.0);
         */
    }

    public void testForwardSwerveRight() {

        autoUtility.setTrajectory(new TrajectoryIterator<>(new TimedView<>(autoUtility.generateTrajectory
                (false, Arrays.asList(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.identity()),
                        new Pose2d(new Translation2d(120.0, -36.0), Rotation2d.identity()),
                        new Pose2d(new Translation2d(240.0, -36.0), Rotation2d.identity())),
                        Arrays.asList(new CentripetalAccelerationConstraint(120.0)),
                        120.0, 120.0, 10.0))));

        double t = 0.0;
        Pose2d pose = autoUtility.setpoint().state().getPose();
        while (!autoUtility.isDone()) {
            autoUtility.update(t, pose);
            //pose = autoUtility.mSetpoint.state().getPose();//.transformBy(new Pose2d(new Translation2d(0.0, 1.0),
            pose = autoUtility.mSetpoint.state().getPose().transformBy(new Pose2d(new Translation2d(0, 0.5), Rotation2d.fromDegrees(0)));
            // Rotation2d.fromDegrees(2.0)));

            System.out.println(t + "," + autoUtility.toCSV());
            System.out.println(autoUtility.error());
            t += 0.01;
        }
    }
}
