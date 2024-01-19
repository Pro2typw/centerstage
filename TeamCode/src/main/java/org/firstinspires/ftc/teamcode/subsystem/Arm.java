package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.justbetter.actuator.CachingDcMotorEX;
import org.firstinspires.ftc.teamcode.subsystem.util.Constants;
import org.firstinspires.ftc.teamcode.subsystem.util.GravityFeedforward;
import org.jetbrains.annotations.NotNull;

public class Arm {

    private final CachingDcMotorEX motor1, motor2;

    private final PIDController differenceController, averageController;

    private GravityFeedforward gravityFeedforward;

    private int extensionTargetPosition = 0;
    private int pivotTargetPosition = 0;
    private int motor1Position, motor2Position;

    private double difference, average, differencePower, averagePower, gravityPower;

    public Arm(HardwareMap hardwareMap) {
        motor1 = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, Constants.Arm.MOTOR1_MAP_NAME));
        motor2 = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, Constants.Arm.MOTOR2_MAP_NAME));

        for(DcMotorEx motor : new DcMotorEx[]{motor1, motor2}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        differenceController = new PIDController(Constants.Arm.AVERAGE_PID_COEFFICIENTS.kP, Constants.Arm.AVERAGE_PID_COEFFICIENTS.kI, Constants.Arm.AVERAGE_PID_COEFFICIENTS.kD);
        averageController = new PIDController(Constants.Arm.DIFFERENCE_PID_COEFFICIENTS.kP, Constants.Arm.DIFFERENCE_PID_COEFFICIENTS.kI, Constants.Arm.DIFFERENCE_PID_COEFFICIENTS.kD);
        gravityFeedforward = new GravityFeedforward(Constants.Arm.GRAVITY_FEEDFORWARD_GAIN);
    }

    public void setGravityFeedforward(@NotNull double gain) {
        gravityFeedforward.setGain(gain);
    }

    public void setDifferenceControllerPID(@NotNull PIDCoefficients pid) {
        differenceController.setPID(pid.kP, pid.kI, pid.kD);
    }

    public void setAverageControllerPID(@NotNull PIDCoefficients pid) {
        averageController.setPID(pid.kP, pid.kI, pid.kD);
    }

    public void setExtensionTargetPosition(@NotNull int targetPosition) {
        extensionTargetPosition = targetPosition;
    }

    public void setPivotTargetPosition(@NotNull int targetPosition) {
        pivotTargetPosition = targetPosition;
    }

    public int getExtensionTargetPosition() {
        return extensionTargetPosition;
    }

    public int getPivotTargetPosition() {
        return pivotTargetPosition;
    }

    public int getPivotCurrentPosition() {
        return (motor1Position + motor2Position)/2;
    }

    public int getExtensionCurrentPosition() {
        return  motor1Position - motor2Position;
    }

    public double getGravityFFPower() {
        return gravityPower;
    }

    public void update() {
        motor1Position = motor1.getCurrentPosition();
        motor2Position = motor2.getCurrentPosition();

        difference = motor1Position - motor2Position; // todo figure out if this is reversed
        average = (double) (motor1Position + motor2Position) / 2.0;

        differencePower = differenceController.calculate(difference, extensionTargetPosition);
        averagePower = averageController.calculate(average, extensionTargetPosition);
        gravityPower = gravityFeedforward.calculate(ticksToMillimeters(difference), Math.toRadians(ticksToDegrees(average)));

        double power1 = differencePower + averagePower + gravityPower;
        double power2 = -differencePower + averagePower - gravityPower;

        motor1.setPower(power1);
        motor2.setPower(power2);
    }

    public static double backdropYtoX(double y) {
        return y / Math.sqrt(3);
    }

    public static double ticksToMillimeters(double ticks) {
        return ticks / 8.94468118871;
    }

    public static int millimetersToTicks(double millimeters) {
        return (int) (8.94468118871 * millimeters);
    }

    public static double ticksToDegrees(double ticks) {
        return ticks / 4.4807486631;
    }

    public static double degreesToTicks(double degrees) {
        return (int) (degrees * 4.4807486631);
    }
}
