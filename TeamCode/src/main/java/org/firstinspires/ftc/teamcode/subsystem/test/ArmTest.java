package org.firstinspires.ftc.teamcode.subsystem.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Arm;

@Disabled
@TeleOp(group = "test")
@Config
public class ArmTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);
        MultipleTelemetry telemetry1 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            arm.update();

            telemetry1.addData("Pivot Current Position", arm.getPivotCurrentPosition());
            telemetry1.addData("Pivot Target Position", arm.getPivotTargetPosition());

            telemetry1.addData("Extension Current Position", arm.getExtensionCurrentPosition());
            telemetry1.addData("Extension Target Position", arm.getExtensionTargetPosition());
        }
    }
}
