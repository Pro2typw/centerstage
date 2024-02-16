package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.util.LoopRateTracker;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;
import org.firstinspires.ftc.teamcode.util.gamepad.JustPressed;

import java.util.concurrent.TimeUnit;

@TeleOp(group = "0", name = "Regionals Game Teleop")
public class Regionals extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = TelemetryUtil.initTelemetry(telemetry);
        Robot robot = new Robot(hardwareMap, telemetry);

        JustPressed jp1 = new JustPressed(gamepad1);
        JustPressed jp2 = new JustPressed(gamepad2);
        LoopRateTracker loopRateTracker = new LoopRateTracker(TimeUnit.MILLISECONDS);

        boolean isSlowMode = false;

        do {
            robot.update();
        } while (opModeInInit());

        waitForStart();

        robot.init();

        while (opModeIsActive()) {

            // hang
            if(jp2.guide()) robot.hang.cycleNextHangState();

            // drone
            if(jp1.guide()) robot.drone.launch();

            // arm


            telemetry.update();
            jp1.update();
            jp2.update();
            loopRateTracker.update();
            robot.update();
            robot.clearCache();
        }
    }
}
