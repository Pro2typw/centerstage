package org.firstinspires.ftc.teamcode.opmode.game.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.util.gamepad.JustPressed;

@TeleOp(group = "0")
public class JudgingTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry);
        robot.wrist.setPosition(Constants.Wrist.INTAKE_POS);
        waitForStart();
        robot.init();

        JustPressed jp = new JustPressed(gamepad1);
        while (opModeIsActive()) {


            if(jp.dpad_up()) robot.arm.setPivotTargetPos(580);
            if(jp.dpad_down()) robot.arm.setPivotTargetPos(150);
            if(jp.dpad_left()) robot.arm.setExtensionTargetPos(0);
            if(jp.dpad_right()) robot.arm.setExtensionTargetPos(200);



//            robot.clearCache();
            jp.update();
            robot.update();
        }
    }
}
