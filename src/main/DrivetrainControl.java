package main;

/**
 * Created by Jody on 10/30/2018.
 */
public class DrivetrainControl {
    private double desiredLeftVelocity = 0;
    private double desiredRightVelocity = 0;

    private double leftFF = 0;
    private double rightFF = 0;

    //output += Kp*e_vel + Ki*e_vel_sum + Kd*e_vel_delta;

    private double pVelocityGain = 0.0005; //tune me!!!!
    private double iVelocityGain = 0;
    private double dVelocityGain = 0;

    private QuickPID leftPID = new QuickPID(pVelocityGain, iVelocityGain, dVelocityGain);
    private QuickPID rightPID = new QuickPID(pVelocityGain, iVelocityGain, dVelocityGain);

    private double kalamnR = 0.05; //tune this

    BadKalman leftKalman = new BadKalman(kalamnR, 0);
    BadKalman rightKalman = new BadKalman(kalamnR, 0);

    public void setDesired(double left, double right, double lFF, double rFF) {
        desiredLeftVelocity = left;
        desiredRightVelocity = right;
        leftFF = lFF;
        rightFF = rFF;
    }

    public void update(double leftVelocity, double rightVelocity) {
        //TODO: INTERNALLY SUPPLY LEFT/RIGHT VELOCITY FROM OUR OWN SENSORS AT RAW UPDATE RATES

        double filteredLeftVelocity = leftKalman.update(leftVelocity);
        double filteredRightVelocity = rightKalman.update(rightVelocity);

        double leftOutput = leftPID.update(filteredLeftVelocity, desiredLeftVelocity) + leftFF;
        double rightOutput = rightPID.update(filteredRightVelocity, desiredRightVelocity) + rightFF;

        System.out.println(filteredLeftVelocity + " " + leftVelocity + " " + desiredLeftVelocity);

        System.out.println("MOTOR OUTPUT: " + leftOutput + " " + rightOutput);
    }

}

class QuickPID {
    private double output = 0;
    private double accumError;
    private double lastError;

    private double p, i, d;

    public QuickPID(double _p, double _i, double _d) {
        p = _p;
        i = _i;
        d = _d;
    }

    public double update(double actual, double setpoint) {
        double error = setpoint - actual;
        output = output + (p * error + (i * accumError) + (d * (error - lastError))); //ignore dt for fun
        lastError = error;
        return output;
    }
}

class BadKalman {

    private final double r; //noise value
    private double x = 0;
    private double p = 0.5; //tune this
    private double k = 0;

    public BadKalman(double _r, double _measured) {
        r = _r;
        x = _measured;
        k = p / (p + r);
    }

    public double update(double measurement) {
        x = x + k * (measurement - x);
        p = 1 - k;
        return x;
    }
}