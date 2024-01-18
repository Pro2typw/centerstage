package org.firstinspires.ftc.teamcode.subsystem.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Arm;

@TeleOp(group = "test")
@Config
public class ArmTest extends LinearOpMode {

    public static double pivotTargetPos = 0;
    public static double extensionTargetPos = 0;

    public static PIDCoefficients average = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients difference = new PIDCoefficients(0, 0, 0);
    public static double gravity = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        waitForStart();
        while (opModeIsActive()) {
            arm.setAverageControllerPID(average);
            arm.setDifferenceControllerPID(difference);
            arm.setGravityFeedforward(gravity);

            arm.update();

            telemetry.addData("Pivot Current Position", arm.getPivotCurrentPosition());
            telemetry.addData("Pivot Target Position", arm.getPivotTargetPosition());

            telemetry.addData("Extension Current Position", arm.getExtensionCurrentPosition());
            telemetry.addData("Extension Target Position", arm.getExtensionTargetPosition());
        }
    }
}
