package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.util.GravityFeedforward;
import org.jetbrains.annotations.NotNull;

public class Arm {
    public enum ArmParts {
        EXTENSION,
        PIVOT
    }

    private final DcMotorEx motor1, motor2;

    private final PIDFController differenceController, averageController;
    private final GravityFeedforward gravityFeedforward;

    private int extensionTargetPosition = 0;
    private int pivotTargetPosition = 0;

    public Arm(HardwareMap hardwareMap) {
        motor1 = hardwareMap.get(DcMotorEx.class, "");
        motor2 = hardwareMap.get(DcMotorEx.class, "");

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
    }

    public int getTargetPosition(@NotNull ArmParts part) {
        return part == ArmParts.EXTENSION ? extensionTargetPosition : pivotTargetPosition;
    }

    public void update() {
        int pos1 = motor1.getCurrentPosition();
        int pos2 = motor2.getCurrentPosition();

        double difference = pos1 - pos2;
        double average = (double) (pos1 + pos2) / 2.0;

        double differencePower = differenceController.update(difference);
        double averagePower = averageController.update(average);
        double gravityPower = gravityFeedforward.calculate(ticksToMeters(difference), ticksToRadians(average));

        double power1 = differencePower + averagePower + gravityPower;
        double power2 = -differencePower + averagePower - gravityPower;

        motor1.setPower(power1);
        motor2.setPower(power2);
    }

    // do some conversion
    public double ticksToMeters(double ticks) {
        return 0;
    }

    // do some conversion
    public double ticksToRadians(double ticks) {
        return 0;
    }
}
