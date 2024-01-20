package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.photoncore.Photon;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Hang;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.HtmlFormatter;
import org.firstinspires.ftc.teamcode.util.LoopRateTracker;
import org.firstinspires.ftc.teamcode.util.gamepad.JustPressed;

@TeleOp(name = "LM3 Game", group = "game")
@Photon
public class LM3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        Robot robot = new Robot(hardwareMap, telemetry, Claw.ClawState.OPEN, Hang.HangState.DOWN, AllianceColor.BLUE);
        JustPressed gp1 = new JustPressed(gamepad1);
        JustPressed gp2 = new JustPressed(gamepad2);

        double power = 4;

        do {
            robot.update();
        } while (opModeInInit());

        waitForStart();


        while (opModeIsActive()) {

            robot.clearCache();
            robot.update();

            robot.drive.setPowers(gp1.left_stick_x(), gp1.left_stick_y(), gp1.right_stick_x(), x -> (Math.pow(x, power) * .75 * Math.signum(x)));

            robot.getTelemetry();
//            robot.telemetry.addData(new HtmlFormatter().textColor("red").format("LoopRate"), loopRateTracker.getLoopTime() + "ms");
            robot.telemetry.update();

            gp1.update();
            gp2.update();
        }
    }
}

/*
    Controls:
    *

*/
