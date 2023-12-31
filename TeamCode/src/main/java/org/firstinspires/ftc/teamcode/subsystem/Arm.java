package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.util.GravityFeedforward;
import org.jetbrains.annotations.NotNull;

public class Arm {
    public static enum ArmParts {
        EXTENSION,
        PIVOT
    }

    private final DcMotorEx motor1, motor2;

    // todo figure out whether difference is extension or vice versa
    private final PIDFController differenceController, averageController;
    private final GravityFeedforward gravityFeedforward;

    private int extensionTargetPosition = 0;
    private int pivotTargetPosition = 0;
    private int motor1Position, motor2Position;

    public Arm(HardwareMap hardwareMap) {
        motor1 = hardwareMap.get(DcMotorEx.class, "");
        motor2 = hardwareMap.get(DcMotorEx.class, "");

        for(DcMotorEx motor : new DcMotorEx[]{motor1, motor2}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // todo config and properly add it to constatnsa and shit
        differenceController = new PIDFController(new PIDCoefficients(0, 0,0));
        averageController = new PIDFController(new PIDCoefficients(0, 0, 0 ));
        gravityFeedforward = new GravityFeedforward(0);

        differenceController.setOutputBounds(0, 1);
        averageController.setOutputBounds(0, 1);
    }

    public void setTargetPosition(@NotNull ArmParts part, @NotNull int targetPosition) {
        if(part == ArmParts.EXTENSION) extensionTargetPosition = targetPosition;
        else pivotTargetPosition = targetPosition;

        differenceController.setTargetPosition(extensionTargetPosition);
        averageController.setTargetPosition(pivotTargetPosition);
    }

    public int getTargetPosition(@NotNull ArmParts part) {
        return part == ArmParts.EXTENSION ? extensionTargetPosition : pivotTargetPosition;
    }

    public int getCurrentPosition(@NotNull ArmParts part) {
        return part == ArmParts.EXTENSION ? motor1Position - motor2Position : (motor1Position + motor2Position)/2;
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

    // do some conversion
    public static double ticksToMeters(double ticks) {
        return ticks;
    }

    // do some conversion
    public static double ticksToRadians(double ticks) {
        return ticks;
    }
}
