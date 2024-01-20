package org.firstinspires.ftc.teamcode.subsystem.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystem.Arm;

@TeleOp(group = "test")
@Config
public class ArmTest extends LinearOpMode {

    public VoltageSensor batterVoltageSensor;

    public static double pivotTargetPos = 0;
    public static double extensionTargetPos = 0;

    public static PIDCoefficients averageCoef = new PIDCoefficients(0.008, 0, 0);
    public static PIDCoefficients differenceCoef = new PIDCoefficients(0.004, 9e-16, 100000);



    public double differenceError = 0;
    public double averageError;
    public double lastdifferenceError = 0;
    public double lastaverageError = 0;
    public double totaldifferenceError = 0;
    public double totalaverageError = 0;

    public double batterComp = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);
        batterVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        batterComp = (12.8 / batterVoltageSensor.getVoltage());



        waitForStart();
        long startTime = System.nanoTime();
        long lastTime = startTime;

        while (opModeIsActive()) {

            double difference = (arm.motor1.getCurrentPosition() - arm.motor2.getCurrentPosition())/2.0;

            differenceError = pivotTargetPos - difference;
            if (differenceError * lastdifferenceError <= 0) totaldifferenceError = 0;
            else totaldifferenceError += differenceError;
            double differenceI = (totaldifferenceError * (System.nanoTime() - startTime)) * differenceCoef.kI;
            double differenceD = ((differenceError - lastdifferenceError) / (System.nanoTime() - lastTime)) * differenceCoef.kD;
            double differenceP = differenceError * differenceCoef.kP;

            double differencePower = differenceP + differenceI + differenceD;



            double average = (arm.motor1.getCurrentPosition() + arm.motor2.getCurrentPosition()) / 2.0;

            averageError = extensionTargetPos - average;
            if (averageError * lastaverageError <= 0) totalaverageError = 0;
            else totalaverageError += averageError;
            double averageI = (totalaverageError * (System.nanoTime() - startTime)) * averageCoef.kI;
            double averageD = ((averageError - lastaverageError) / (System.nanoTime() - lastTime)) * averageCoef.kD;
            double averageP = averageError * averageCoef.kP;

            double averagePower = averageP + averageI + averageD;
            double gravityPower = Math.cos(Math.toRadians(Arm.ticksToDegrees(difference))) * .15 * batterComp;
            double power1 = differencePower + averagePower + gravityPower;
            double power2 = -differencePower + averagePower - gravityPower;

            arm.motor1.setPower(power1);
            arm.motor2.setPower(power2);



            telemetry.addData("Pivot Current Position", difference);
            telemetry.addData("Pivot Target Position", pivotTargetPos);
            telemetry.addData("kP", differenceP);
            telemetry.addData("kI", differenceI);
            telemetry.addData("kD", differenceD);

//            telemetry.addData("Extension Current Position", arm.getExtensionCurrentPosition());
//            telemetry.addData("Extension Target Position (mm)", Arm.ticksToMillimeters(arm.getExtensionCurrentPosition()));
//            telemetry.addData("Extension Target Position", arm.getExtensionTargetPosition());
//            telemetry.addLine();
//            telemetry.addData("Gravity FF Gain", gravity);
//            telemetry.addData("Gravity FF Power", arm.getGravityFFPower());
//            telemetry.addLine();
            telemetry.addData("Left Motor Position", arm.motor1.getCurrentPosition());
            telemetry.addData("Right Motor Position", arm.motor2.getCurrentPosition());
            telemetry.update();

            lastTime = System.nanoTime();
            lastdifferenceError = differenceError;
        }
    }
}
