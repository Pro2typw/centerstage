package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.control.PIDFController;
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
    private final PIDFController differenceController, averageController;
    private final GravityFeedforward gravityFeedforward;

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

        differenceController = new PIDFController(Constants.Arm.AVERAGE_PID_COEFFICIENTS);
        averageController = new PIDFController(Constants.Arm.DIFFERENCE_PID_COEFFICIENTS);
        gravityFeedforward = new GravityFeedforward(Constants.Arm.GRAVITY_FEEDFORWARD_GAIN);

        differenceController.setOutputBounds(0, 1);
        averageController.setOutputBounds(0, 1);
    }

    public void setExtensionTargetPosition(@NotNull int targetPosition) {
        extensionTargetPosition = targetPosition;
        differenceController.setTargetPosition(extensionTargetPosition);
    }

    public void setPivotTargetPosition(@NotNull int targetPosition) {
        pivotTargetPosition = targetPosition;
        averageController.setTargetPosition(pivotTargetPosition);
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

        double differencePower = differenceController.update(difference);
        double averagePower = averageController.update(average);
        double gravityPower = gravityFeedforward.calculate(ticksToMeters(difference), ticksToRadians(average));

        double power1 = differencePower + averagePower + gravityPower;
        double power2 = -differencePower + averagePower - gravityPower;

        motor1.setPower(power1);
        motor2.setPower(power2);
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
