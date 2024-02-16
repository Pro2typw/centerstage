package org.firstinspires.ftc.teamcode.subsystem.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Wrist;
import org.firstinspires.ftc.teamcode.subsystem.util.Constants;
import org.firstinspires.ftc.teamcode.util.WPIMathUtil;

@TeleOp(group = "test")
@Config
public class ArmPIDTuning extends LinearOpMode {

    public VoltageSensor batterVoltageSensor;


    public static double pivotTargetPos = 0;
    public static double extensionTargetPos = 0;

    public static PIDCoefficients averageCoef = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients differenceCoef = new PIDCoefficients(0.0036, 2.5e-12, 170000);



    public double differenceError = 0;
    public double averageError;
    public double lastdifferenceError = 0;
    public double lastaverageError = 0;
    public double totaldifferenceError = 0;
    public double totalaverageError = 0;

    public double batterComp = 0;

    public static double G = 1.4;

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);
//        Wrist wrist = new Wrist(hardwareMap);
        batterVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        batterComp = (12.8 / batterVoltageSensor.getVoltage());



        waitForStart();
        long startTime = System.nanoTime();
        long lastTime = startTime;

        while (opModeIsActive()) {
            long time = System.nanoTime() - startTime;
//            wrist.setPosition(Constants.Wrist.INTAKE_POS);
            double difference = (arm.motor1.getCurrentPosition() - arm.motor2.getCurrentPosition())/2.0;

            differenceError = pivotTargetPos - difference;
            if (differenceError * lastdifferenceError <= 0 || Math.abs(differenceError - lastdifferenceError) > 5) totaldifferenceError = 0;
            else totaldifferenceError += differenceError;

            double differenceI = (totaldifferenceError * (time - lastTime)) * differenceCoef.kI;
            double differenceD = ((differenceError - lastdifferenceError) / (time - lastTime)) * differenceCoef.kD;
            double differenceP = differenceError * differenceCoef.kP;

            double differencePower = differenceP + differenceI + differenceD;

            double average = (arm.motor1.getCurrentPosition() + arm.motor2.getCurrentPosition()) / 2.0;

            averageError = extensionTargetPos - average;
            if (averageError * lastaverageError <= 0 || Math.abs(averageError - lastaverageError) > 5) totalaverageError = 0;
            else totalaverageError += averageError;
            double averageI = (totalaverageError * (time - lastTime)) * averageCoef.kI;
            double averageD = ((averageError - lastaverageError) / (time - lastTime)) * averageCoef.kD;
            double averageP = averageError * averageCoef.kP;

            double averagePower = averageP + averageI + averageD;
            double gravityPower = difference > 20 ? Math.cos(Math.toRadians(Arm.ticksToDegrees(difference))) * .15 * batterComp * G : 0;
            double power1 = differencePower + averagePower + gravityPower;
            double power2 = -differencePower + averagePower - gravityPower;

            arm.motor1.setPower(power1);
            arm.motor2.setPower(power2);

            telemetry.addData("Pivot Current Position", difference);
            telemetry.addData("Pivot Target Position", pivotTargetPos);
            telemetry.addData("Pivot Error", pivotTargetPos - difference);
            telemetry.addData("Pivot Current Position (deg)", Arm.ticksToDegrees(difference));
            telemetry.addData("Difference kP", differenceP);
            telemetry.addData("Difference kI", differenceI);
            telemetry.addData("Difference kD", differenceD);
            telemetry.addData("Average kP", averageP);
            telemetry.addData("Average kI", averageI);
            telemetry.addData("Average kD", averageD);

            telemetry.addData("G", G);
            telemetry.addData("Left Motor Position", arm.motor1.getCurrentPosition());
            telemetry.addData("Right Motor Position", arm.motor2.getCurrentPosition());
            telemetry.addData("Extension Current Position", average);
//            telemetry.addData("Extension Target Position (mm)", Arm.ticksToMillimeters(arm.getExtensionCurrentPosition()));
            telemetry.addData("Extension Target Position", extensionTargetPos);
            telemetry.addData("Extension Error", extensionTargetPos - average);
            telemetry.addData("delta time", time - lastTime);
            telemetry.addData("Last Extension Error", lastaverageError);
//            telemetry.addLine();
//            telemetry.addData("Gravity FF Gain", gravity);
//            telemetry.addData("Gravity FF Power", arm.getGravityFFPower());
//            telemetry.addLine();
            telemetry.update();

            lastTime = time;
            lastdifferenceError = differenceError;
            lastaverageError = averageError;
        }
    }
}
