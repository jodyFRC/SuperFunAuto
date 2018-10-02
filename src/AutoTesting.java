import lib.geometry.Pose2d;
import lib.geometry.Rotation2d;
import lib.geometry.Translation2d;
import lib.trajectory.TimedView;
import lib.trajectory.TrajectoryIterator;
import lib.trajectory.timing.CentripetalAccelerationConstraint;

import java.util.Arrays;

/**
 * Created by Jody on 10/2/2018.
 */
public class AutoTesting {

    public static void main(String[] args) {
        AutoTesting test = new AutoTesting();
        test.testForwardSwerveRight();
    }

    public void testForwardSwerveRight() {
        SuperFunAutoUtility autoUtility = new SuperFunAutoUtility();
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
