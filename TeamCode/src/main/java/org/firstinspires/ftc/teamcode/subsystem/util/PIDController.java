package org.firstinspires.ftc.teamcode.subsystem.util;

// todo why have this when it already exists..
public class PIDController {
    private final double kP;
    private final double kI;
    private final double kD;
    private final double targetValue;
    private final double windUpPreventionTolerance;
    private final double timestamp;

    private double error;
    private double prevError;
    private double avgErrorChange;

    private double errorSum;

    private double power;

    public PIDController(double kP, double kI, double kD, double targetValue, double windUpPreventionTolerance, double timestamp) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.targetValue = targetValue;
        this.windUpPreventionTolerance = windUpPreventionTolerance;
        this.timestamp = timestamp;
        errorSum = 0;
        prevError = 0;
        avgErrorChange = 0;
    }

    public double calculate(double currentValue, double currentTimeStamp) {
        error = targetValue - currentValue;

        if (Math.abs(error) > windUpPreventionTolerance) errorSum += error;
        else errorSum = 0;

        avgErrorChange = (error - prevError) / (currentTimeStamp - timestamp);

        double P = kP * error;
        double I = kI * errorSum;
        double D = kD * avgErrorChange;

        power = P + I + D;

        return power;
    }

    public double getPower() {
        return power;
    }

}
