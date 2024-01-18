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

    // todo figure out whether difference is extension or vice versa
    private final PIDController differenceController, averageController;

    private GravityFeedforward gravityFeedforward;

    private int extensionTargetPosition = 0;
    private int pivotTargetPosition = 0;
    private int motor1Position, motor2Position;

    public Arm(HardwareMap hardwareMap) {
        motor1 = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, Constants.Arm.MOTOR1_MAP_NAME));
        motor2 = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, Constants.Arm.MOTOR2_MAP_NAME));

        for(DcMotorEx motor : new DcMotorEx[]{motor1, motor2}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        differenceController = new PIDController(Constants.Arm.AVERAGE_PID_COEFFICIENTS.kP, Constants.Arm.AVERAGE_PID_COEFFICIENTS.kI, Constants.Arm.AVERAGE_PID_COEFFICIENTS.kD);
        averageController = new PIDController(Constants.Arm.DIFFERENCE_PID_COEFFICIENTS.kP, Constants.Arm.DIFFERENCE_PID_COEFFICIENTS.kI, Constants.Arm.DIFFERENCE_PID_COEFFICIENTS.kD);
        gravityFeedforward = new GravityFeedforward(Constants.Arm.GRAVITY_FEEDFORWARD_GAIN);
    }

    public void setGravityFeedforward(@NotNull double gain) {
        gravityFeedforward = new GravityFeedforward(gain);
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


    public void update() {
        motor1Position = motor1.getCurrentPosition();
        motor2Position = motor2.getCurrentPosition();

        double difference = motor1Position - motor2Position; // todo figure out if this is reversed
        double average = (double) (motor1Position + motor2Position) / 2.0;

        double differencePower = differenceController.calculate(difference, extensionTargetPosition);
        double averagePower = averageController.calculate(average, extensionTargetPosition);
        double gravityPower = gravityFeedforward.calculate(ticksToMeters(difference), ticksToRadians(average));

        double power1 = differencePower + averagePower + gravityPower;
        double power2 = -differencePower + averagePower - gravityPower;

        motor1.setPower(power1);
        motor2.setPower(power2);
    }

    public static double backdropYtoX(double y) {
        return y / Math.sqrt(3);
    }

    // todo do some conversion
    public static double ticksToMeters(double ticks) {
        return ticks;
    }

    public static int metersToTicks(double meters) {
        return (int) meters;
    }

    public static double ticksToRadians(double ticks) {
        return ticks;
    }

    public static double radiansToTicks(double radians) {
        return (int) radians;
    }
}
