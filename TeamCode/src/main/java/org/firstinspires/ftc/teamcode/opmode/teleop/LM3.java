package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Hang;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.util.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystem.util.Constants;
import org.firstinspires.ftc.teamcode.util.HtmlFormatter;
import org.firstinspires.ftc.teamcode.util.LoopRateTracker;
import org.firstinspires.ftc.teamcode.util.gamepad.JustPressed;
import org.firstinspires.ftc.teamcode.subsystem.Launch;

@TeleOp(name = "LM3 Game", group = "game")
public class LM3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LoopRateTracker loopRateTracker;
        Robot robot = new Robot(hardwareMap, telemetry, Claw.ClawState.OPEN, Hang.HangState.DOWN, true, AllianceColor.BLUE);
        JustPressed gp1 = new JustPressed(gamepad1);
        JustPressed gp2 = new JustPressed(gamepad2);

        JustPressed jpgamepad1 = new JustPressed(gamepad1);
        JustPressed jpgamepad2 = new JustPressed(gamepad2);


        double power = 4;

        do {
            robot.update();
        } while (opModeInInit());

        waitForStart();

        loopRateTracker = new LoopRateTracker();

        while (opModeIsActive()) {
            robot.clearCache();
            robot.update();
            loopRateTracker.update();

            robot.drive.setPowers(gp1.left_stick_x(), gp1.left_stick_y(), gp1.right_stick_x(), x -> (Math.pow(x, power) * .75 * Math.signum(x)));

            //Plane
            if(jpgamepad1.x()) robot.launch.switchState();

            //Claw
            if(jpgamepad1.left_bumper() || jpgamepad2.left_bumper()) //robot.claw.inverseLeftClawState();
            if(jpgamepad1.right_bumper() || jpgamepad2.right_bumper()) //robot.claw.inverseRightClawState();
            if(jpgamepad1.b() || jpgamepad2.b()) {
                //robot.claw.inverseLeftClawState();
                //robot.claw.inverseRightClawState();

            //arm stuff
            if(jpgamepad1.a()) {
                //ashwin press A, (if already at backdrop -> goes to floor) and vice versa
    //            if (robot.arm.isAtBackdrop()) {
    //                robot.arm.moveToFloor();
                } else {
    //                robot.arm.moveToBackdrop();
                }
            }

            //manual adjust for pivot - left stick gamepad 2
  //          robot.arm.adjustPivot(jpgamepad2.left_stick_y());

            //manual adjust for wrist - right stick gamepad 2
  //          robot.arm.adjustWrist(jpgamepad2.right_stick_y());

            }





            robot.getTelemetry();
            robot.telemetry.addData(new HtmlFormatter().textColor("red").format("LoopRate"), loopRateTracker.getLoopTime() + "ms");
            robot.telemetry.update();

            gp1.update();
            gp2.update();
        }
    }
}

/*
    Controls:
    *

    Plane - driver 1: x
    open and close claw: bumpers (individual), both : B
    ashwin press A, (if already at backdrop -> goes to floor) and vice versa
    manual adjust for pivot - left stick gamepad 2
    manual adjust for wrist - right stick gamepad 2


*/
