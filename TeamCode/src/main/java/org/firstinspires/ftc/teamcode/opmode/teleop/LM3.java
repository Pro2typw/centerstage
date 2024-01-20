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
import org.firstinspires.ftc.teamcode.subsystem.Wrist;
import org.firstinspires.ftc.teamcode.subsystem.util.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystem.util.Constants;
import org.firstinspires.ftc.teamcode.util.HtmlFormatter;
import org.firstinspires.ftc.teamcode.util.LoopRateTracker;
import org.firstinspires.ftc.teamcode.util.WPIMathUtil;
import org.firstinspires.ftc.teamcode.util.gamepad.JustPressed;
import org.firstinspires.ftc.teamcode.subsystem.Launch;

@TeleOp(name = "LM3 Game", group = "game")
public class LM3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry, Claw.ClawState.OPEN, Hang.HangState.DOWN);
        JustPressed gp1 = new JustPressed(gamepad1);
        JustPressed gp2 = new JustPressed(gamepad2);

        JustPressed jpgamepad1 = new JustPressed(gamepad1);
        JustPressed jpgamepad2 = new JustPressed(gamepad2);


        double power = 4;

        do {
            robot.update();
        } while (opModeInInit());

        waitForStart();

        robot.init();

        while (opModeIsActive()) {
            robot.clearCache();
            robot.update();

            robot.drive.setPowers(gp1.left_stick_x(), -gp1.left_stick_y(), gp1.right_stick_x(), x -> (Math.pow(x, power) * .75 * Math.signum(x)));

            //Plane
            if(jpgamepad1.x()) robot.launch.launch();

            //Claw
            if(jpgamepad1.left_bumper() || jpgamepad2.left_bumper()) robot.claw.setClawState(Claw.ClawSide.LEFT, robot.claw.getClawState(Claw.ClawSide.LEFT) == Claw.ClawState.CLOSE ? Claw.ClawState.OPEN : Claw.ClawState.CLOSE);
            if(jpgamepad1.right_bumper() || jpgamepad2.right_bumper()) robot.claw.setClawState(Claw.ClawSide.RIGHT, robot.claw.getClawState(Claw.ClawSide.RIGHT) == Claw.ClawState.CLOSE ? Claw.ClawState.OPEN : Claw.ClawState.CLOSE);
            if(jpgamepad1.b() || jpgamepad2.b()) {
                robot.claw.setClawState(Claw.ClawSide.LEFT, robot.claw.getClawState(Claw.ClawSide.LEFT) == Claw.ClawState.CLOSE ? Claw.ClawState.OPEN : Claw.ClawState.CLOSE);
                robot.claw.setClawState(Claw.ClawSide.RIGHT, robot.claw.getClawState(Claw.ClawSide.RIGHT) == Claw.ClawState.CLOSE ? Claw.ClawState.OPEN : Claw.ClawState.CLOSE);
            }

            //arm stuff
            if(jpgamepad1.a()) {
                robot.arm.setPivotTargetPos(WPIMathUtil.isNear(600, robot.arm.getPivotTargetPos(), 20) ? 100 : 600);
            }

            //manual adjust for pivot - left stick gamepad 2
  //          robot.arm.adjustPivot(jpgamepad2.left_stick_y());
            if(robot.arm.getPivotTargetPos() + jpgamepad2.left_stick_y() * 10 < 600 || robot.arm.getPivotTargetPos() + jpgamepad2.left_stick_y() * 10 > 0)
                robot.arm.setPivotTargetPos(robot.arm.getPivotTargetPos() - jpgamepad2.left_stick_y() * 10);
            //manual adjust for wrist - right stick gamepad 2
  //          robot.arm.adjustWrist(jpgamepad2.right_stick_y());
            if(jpgamepad2.dpad_down()) robot.wrist.setState(Wrist.WristState.LOW_DEPO_POS);
            if(jpgamepad2.dpad_up()) robot.wrist.setState(Wrist.WristState.HIGH_DEPO_POS);
            if(jpgamepad2.dpad_left()) robot.wrist.setState(Wrist.WristState.INTAKE_POS);

            jpgamepad2.update();
            jpgamepad1.update();

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
