package org.firstinspires.ftc.teamcode.subsystem.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Drone;
import org.firstinspires.ftc.teamcode.subsystem.Wrist;
import org.firstinspires.ftc.teamcode.subsystem.util.Constants;
import org.firstinspires.ftc.teamcode.util.LoopRateTracker;
import org.firstinspires.ftc.teamcode.util.gamepad.JustPressed;

import java.util.concurrent.TimeUnit;

@TeleOp(group = "test")
public class LaunchTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        JustPressed jp = new JustPressed(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drone drone;
        drone = new Drone(hardwareMap);
        waitForStart();
        ElapsedTime oneSec = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        boolean done = true;
        while (opModeIsActive()) {

            if(jp.a() && !drone.isLaunched()) drone.launch();
            if(drone.isLaunched() && done) {
                oneSec.reset();
                done = false;
            }
            if(oneSec.now(TimeUnit.MILLISECONDS) > 1000 && !done && drone.isLaunched()) {
                drone.launch();
                done = true;
            }
            telemetry.addLine("Press A to launch (toggles so press again to reset");
            telemetry.addData("LAUNCH MODE", drone.isLaunched() ? "LAUNCHED" : "READY");
            telemetry.update();
            jp.update();
        }


    }
}
