package org.firstinspires.ftc.teamcode.opmode.util;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Wrist;
import org.firstinspires.ftc.teamcode.util.gamepad.JustPressed;

@TeleOp(group = "test")
public class WristTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Wrist wrist = new Wrist(hardwareMap, 0);
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap, Claw.ClawState.OPEN);
        JustPressed jp = new JustPressed(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        wrist.setPosition(Constants.Wrist.INTAKE_POS);
        waitForStart();
        arm.init();
        while (opModeIsActive()) {
            if(jp.left_bumper()) claw.toggleClawState(Claw.ClawSide.BOTH);
            if(jp.y()) arm.setPivotTargetPos(635);
            else if(jp.x()) arm.setPivotTargetPos(500);
            else if(jp.b()) arm.setPivotTargetPos(200);
            else if(jp.a()) arm.setPivotTargetPos(0);
            wrist.setPosition(wrist.getPosition() + .001 * gamepad1.left_stick_y);
            telemetry.addData("Wrist Position", wrist.getPosition());
            telemetry.addData("Pivot Position", arm.getPivotCurrentPos());
            telemetry.update();


            jp.update();
            arm.update();
        }
    }
}