package org.firstinspires.ftc.teamcode.opmode.util;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.util.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.util.justbetter.sensor.Encoder;

@TeleOp(group = "test")
@Config
public class ArmPIDTuner extends LinearOpMode {

    public static PIDCoefficients AVERAGE_PID_COEFFICIENTS = new PIDCoefficients(0.01, 2.5e-11, 10000);
        public static PIDCoefficients DIFFERENCE_PID_COEFFICIENTS = new PIDCoefficients(0.0036, 2.5e-12, 170000);
    public VoltageSensor batterVoltageSensor;
    public static double pivotTargetPos = 0;
    public static double extensionTargetPos = 0;




    public double differenceError = 0;
    public double averageError;
    public double lastdifferenceError = 0;
    public double lastaverageError = 0;
    public double totaldifferenceError = 0;
    public double totalaverageError = 0;

    public double batterComp = 0;

    public static double G_PIVOT = 2.05652575;
    public static double G_PIVOT2 = .23;
    public static double G_EXTENSION = .01;
    public static double G_STATIC = 0.000007;

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);
//        Wrist wrist = new Wrist(hardwareMap);
        batterVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        batterComp = (12.8 / batterVoltageSensor.getVoltage());

        Encoder encoder = new Encoder(hardwareMap.get(DcMotor.class, "fr"));
        encoder.reset();


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

            double differenceI = (totaldifferenceError * (time - lastTime)) * DIFFERENCE_PID_COEFFICIENTS.kI;
            double differenceD = ((differenceError - lastdifferenceError) / (time - lastTime)) * DIFFERENCE_PID_COEFFICIENTS.kD;
            double differenceP = differenceError * DIFFERENCE_PID_COEFFICIENTS.kP;

            double differencePower = Math.abs(differenceError) < 8 ? 0 : differenceP + differenceI + differenceD;

            double average = (arm.motor1.getCurrentPosition() + arm.motor2.getCurrentPosition()) / 2.0;

            averageError = extensionTargetPos - average;
            if (Math.abs(averageError) < 0)
                averageError = 0;
            if (averageError * lastaverageError <= 0 || Math.abs(averageError - lastaverageError) > 5) totalaverageError = 0;
            else totalaverageError += averageError;
            double averageI = (totalaverageError * (time - lastTime)) * AVERAGE_PID_COEFFICIENTS.kI;
            double averageD = ((averageError - lastaverageError) / (time - lastTime)) * AVERAGE_PID_COEFFICIENTS.kD;
            double averageP = averageError * AVERAGE_PID_COEFFICIENTS.kP;

            G_PIVOT = 1 + (G_STATIC * Math.pow(average, 2));

            double averagePower = averageP + averageI + averageD;
            averagePower += G_EXTENSION * Math.sin(Math.toRadians(Arm.ticksToDegrees(difference)));

            double gravityPower = difference > 0 ? Math.cos(Math.toRadians(Arm.ticksToDegrees(difference))) * G_PIVOT2 * batterComp * G_PIVOT: 0;

            double power1, power2;

            if (Math.abs(differencePower) < .2) {
                 power1 = differencePower + averagePower + gravityPower;
                 power2 = -differencePower + averagePower- gravityPower;
            } else {
                 power1 = differencePower + gravityPower;
                 power2 = -differencePower - gravityPower;
            }

            arm.motor1.setPower(power1);
            arm.motor2.setPower(power2);

            telemetry.addData("pivot gravity power", gravityPower);
            telemetry.addData("Pivot Current Position", difference);
            telemetry.addData("Pivot Target Position", pivotTargetPos);
            telemetry.addData("Pivot Error", pivotTargetPos - difference);
            telemetry.addData("Pivot Current Position (deg)", Arm.ticksToDegrees(difference));
            telemetry.addData("Pivot Target Position (deg)", Arm.ticksToDegrees(pivotTargetPos));
            telemetry.addData("motor 1 power", power1);
            telemetry.addData("motor 2 power", power2);
            telemetry.addData("Difference kP", differenceP);
            telemetry.addData("Difference kI", differenceI);
            telemetry.addData("Difference kD", differenceD);
            telemetry.addData("Average kP", averageP);
            telemetry.addData("Average kI", averageI);
            telemetry.addData("Average kD", averageD);

            telemetry.addData("G Pivot", G_PIVOT);
            telemetry.addData("Left Motor Position", arm.motor1.getCurrentPosition());
            telemetry.addData("Right Motor Position", arm.motor2.getCurrentPosition());
            telemetry.addData("Extension Current Position", average);
//            telemetry.addData("Extension Target Position (mm) (with motor encoders)", Arm.ticksToMillimeters(arm.getExtensionCurrentPos()));
//            telemetry.addData("Extension Target Position (mm) (with thru-bore encoder)", Arm.encoderTicksToMillimeters(arm.getCurrentExtensionEncoderPosition()));
            telemetry.addData("Extension Target Position", extensionTargetPos);
            telemetry.addData("Extension Error", extensionTargetPos - average);
            telemetry.addData("delta time", time - lastTime);
            telemetry.addData("Last Extension Error", lastaverageError);
            telemetry.addData("Encoder Extension Current Position", encoder.getCurrentPosition());
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
