package org.firstinspires.ftc.teamcode.subsystem.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Wrist;
import org.firstinspires.ftc.teamcode.subsystem.util.Constants;
import org.firstinspires.ftc.teamcode.util.LoopRateTracker;
import org.firstinspires.ftc.teamcode.util.gamepad.JustPressed;

import java.util.concurrent.TimeUnit;

@TeleOp(group = "test")
public class WristTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Wrist wrist = new Wrist(hardwareMap, 0);
        Arm arm = new Arm(hardwareMap);
        JustPressed jp = new JustPressed(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        LoopRateTracker loopRateTracker = new LoopRateTracker(TimeUnit.MILLISECONDS);

        wrist.setPosition(Constants.Wrist.INTAKE_POS);
        waitForStart();
        arm.init();
        while (opModeIsActive()) {
            loopRateTracker.update();
            if(jp.y()) arm.setPivotTargetPos(580);
            else if(jp.x()) arm.setPivotTargetPos(500);
            else if(jp.b()) arm.setPivotTargetPos(200);
            else if(jp.a()) arm.setPivotTargetPos(0);
            wrist.setPosition(wrist.getPosition() + .001 * gamepad1.left_stick_y);
            telemetry.addData("Wrist Position", wrist.getPosition());
            telemetry.addData("Loop Rate", loopRateTracker.getLoopTime());
            telemetry.update();


            jp.update();
            arm.update();
        }
    }
}
