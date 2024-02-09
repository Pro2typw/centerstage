package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.photoncore.Photon;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.util.LoopRateTracker;
import org.firstinspires.ftc.teamcode.util.WPIMathUtil;
import org.firstinspires.ftc.teamcode.util.gamepad.JustPressed;

@Photon
@TeleOp(name = "League Tournament Game Teleop")
public class JudgingTeleop extends LinearOpMode {

    public static double DRIVETRAIN_POWER = .5;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, telemetry, Claw.ClawState.CLOSE);

        JustPressed gp1 = new JustPressed(gamepad1);
        JustPressed gp2 = new JustPressed(gamepad2);

        do {
            robot.update();
        } while (opModeInInit());
        waitForStart();
        robot.init();
        robot.setArmState(Robot.ArmState.INTAKE);
        while (opModeIsActive()) {

            //pivot
            if(gp1.x()) {
                robot.arm.setPivotTargetPos(robot.arm.getPivotTargetPos() == 500 ? 0 : 500);
            }
            //extension
            if(gp1.y()) robot.arm.setExtensionTargetPos(robot.arm.getExtensionTargetPos() == 500 ? 0 : 500);

            //hang
            if(gp1.dpad_up()) {
                robot.setArmState(Robot.ArmState.INIT);
                robot.hang.setPower(gp2.left_stick_y() * -0.5);
            }

            robot.clearCache();
            robot.update();


            telemetry.addLine("Press X for Pivot");
            telemetry.addLine("Press Y for Extension");

            telemetry.addData("Pivot Current Pos", robot.arm.getPivotCurrentPos());
            telemetry.addData("Pivot Target Pos", robot.arm.getPivotTargetPos());
            telemetry.addData("Extension Current Pos", robot.arm.getExtensionCurrentPos());
            telemetry.addData("Extension Target Pos", robot.arm.getExtensionTargetPos());

            telemetry.update();



            gp1.update();
            gp2.update();
        }
    }
}

/*
 *
 * left/right bumpers for left/right claw to either d1 or d2 (not both)
 * b for both claw to either d1 or d2 (not both)
 * guide for launch for d2:
 *  d2 holds a misc. button and then d1 shoots (safety switch) -- done
 * start for hang for d2 -- done
 *
 * arm stuff:
 *  sequences:
 *   (intake) wait for claw input (only d2 can intake with claw)
 *   (moving between stack/human player to backdrop) wrist and pivot go into pre position
 *   (deposit) extension, pivot and arm work for placing pixel on backdrop (only d1 can depo with claw) (flip claw direction due to robot being 180deg)
 *   ...
 *
 *  controls:
 *   dpad up for next sequence
 *   dpad down for previous sequence
 *   claw position:
 *    dpad left for both claw waiting
 *    left bumper for left claw waiting
 *    right bumper for right claw waiting
 *   dpad right for backdrop pos
 *
 */
