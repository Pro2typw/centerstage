package org.firstinspires.ftc.teamcode.subsystem.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.util.Constants;
import org.firstinspires.ftc.teamcode.subsystem.util.PIDController;

@TeleOp(group = "test")
@Config
public class ArmPIDTuningBetta extends LinearOpMode {

    public static double pivotTargetPos = 0;
    public static double extensionTargetPos = 0;
    public double batterComp = 1;

    @Override
    public void runOpMode() {
        Arm arm = new Arm(hardwareMap);
        VoltageSensor batterVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        batterComp = (Constants.Arm.VOLTAGE_COMPENSATION_CONSTANT / batterVoltageSensor.getVoltage());
        PIDController differenceController;
        PIDController averageController;

        waitForStart();
        differenceController = new PIDController(Constants.Arm.DIFFERENCE_PID_COEFFICIENTS);
        averageController = new PIDController(Constants.Arm.AVERAGE_PID_COEFFICIENTS);

        differenceController.init();
        averageController.init();

        while (opModeIsActive()) {
            differenceController.setTargetPosition(pivotTargetPos);
            averageController.setTargetPosition(extensionTargetPos);

            double difference = (arm.motor1.getCurrentPosition() - arm.motor2.getCurrentPosition()) / 2.0;
            double average = (arm.motor1.getCurrentPosition() + arm.motor2.getCurrentPosition()) / 2.0;

            double differencePower = differenceController.calculate(difference);
            double averagePower = averageController.calculate(average);

            double gravityPower = Math.cos(Math.toRadians(Arm.ticksToDegrees(difference))) * .15 * batterComp;
            double power1 = differencePower + averagePower + gravityPower;
            double power2 = -differencePower + averagePower - gravityPower;

            arm.motor1.setPower(power1);
            arm.motor2.setPower(power2);

            telemetry.addData("Pivot Current Position", differenceController);
            telemetry.addData("Pivot Target Position", pivotTargetPos);
            telemetry.addData("Pivot Error", Math.abs(differenceController.getError()));
            telemetry.addData("Difference kP", differenceController.getP());
            telemetry.addData("Difference kI", differenceController.getI());
            telemetry.addData("Difference kD", differenceController.getD());

            telemetry.addData("Extension Current Position", averageController.getCurrentPosition());
            telemetry.addData("Extension Target Position", extensionTargetPos);
            telemetry.addData("Extension Error", Math.abs(averageController.getError()));
            telemetry.addData("Average kP", averageController.getP());
            telemetry.addData("Average kI", averageController.getI());
            telemetry.addData("Average kD", averageController.getD());

            telemetry.addData("Left Motor Position", arm.motor1.getCurrentPosition());
            telemetry.addData("Right Motor Position", arm.motor2.getCurrentPosition());
            telemetry.addData("Left Motor Power", arm.motor1.getPower());
            telemetry.addData("Right Motor Power", arm.motor2.getPower());

            telemetry.update();
        }
    }
}