package org.firstinspires.ftc.teamcode.subsystem.util;

import com.acmerobotics.roadrunner.control.PIDCoefficients;

import org.firstinspires.ftc.teamcode.util.WPIMathUtil;

public class PIDController {
    private PIDCoefficients pidCoefficients;

    private double targetPosition = 0;
    private double currentPosition = 0;

    private double lastError = 0;
    private double totalError = 0;
    private double error = 0;

    private double minOutput = Integer.MIN_VALUE;
    private double maxOutput = Integer.MAX_VALUE;

    private double startTime;
    private double lastTime;

    double P = 0;
    double I = 0;
    double D = 0;

    /**
     * Constructor
     *
     * @param pidCoefficients PID coefficients for the controller
     */
    public PIDController(PIDCoefficients pidCoefficients) {
        this.pidCoefficients = pidCoefficients;
    }

    /**
     * Initialize the PID controller
     */
    public void init() {
        startTime = System.nanoTime();
        lastTime = startTime;
    }

    /**
     * Set the target position for the controller
     *
     * @param targetPosition The desired target position
     */
    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    /**
     * Set new PID coefficients for the controller
     *
     * @param pidCoefficients New PID coefficients
     */
    public void setPidCoefficients(PIDCoefficients pidCoefficients) {
        this.pidCoefficients = pidCoefficients;
    }

    /**
     * Calculate the control output based on the current position
     *
     * @param currentPosition The current position of the controlled system
     * @return The computed control output
     */
    public double calculate(double currentPosition) {
        this.currentPosition = currentPosition;
        error = targetPosition - currentPosition;

        if (error * lastError <= 0) totalError = 0;
        else totalError += error;

        P = error * pidCoefficients.kP;
        I = (totalError * (System.nanoTime() - lastTime)) * pidCoefficients.kI;
        D = ((error - lastError) / (System.nanoTime() - lastTime)) * pidCoefficients.kD;

        double power = P + I + D;

        lastTime = System.nanoTime();
        lastError = error;

        return WPIMathUtil.clamp(power, minOutput, maxOutput);
    }

    public void reset() {
        totalError = 0;
        lastError = 0;
        init();
    }

    public PIDCoefficients getPidCoefficients() {
        return pidCoefficients;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getCurrentPosition() {
        return currentPosition;
    }

    public double getLastError() {
        return lastError;
    }

    public double getTotalError() {
        return totalError;
    }

    public double getError() {
        return error;
    }

    public double getP() {
        return P;
    }

    public double getI() {
        return I;
    }

    public double getD() {
        return D;
    }
}