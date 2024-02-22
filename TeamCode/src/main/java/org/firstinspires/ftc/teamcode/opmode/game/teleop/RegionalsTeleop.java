package org.firstinspires.ftc.teamcode.opmode.game.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.photoncore.Photon;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.util.gamepad.JustPressed;

@Photon
@Config
@TeleOp(group = "0")
public class RegionalsTeleop extends LinearOpMode {
    public static int maxExtension = 300;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, telemetry, Claw.ClawState.OPEN);
        JustPressed jp1 = new JustPressed(gamepad1);
        JustPressed jp2 = new JustPressed(gamepad2);

        /*
        0: intake; wrist at intake
        1: transition; pivot at 580
        2: depo; no extension pivot at 580
         */
        int state = 0;
        final int maxStates = 3;

        int ticksPerMajor = 200;
        int ticksPerMinor = 100;


        boolean isSlowMode = false;

        boolean isHang = false;
        boolean justHung = false;


        robot.update();

        do {
            robot.update();
        } while (opModeInInit() && !isStopRequested());

        robot.wrist.setPosition(Constants.Wrist.INTAKE_POS);

        waitForStart();

        robot.init();

        while (opModeIsActive() && !isStopRequested()) {


            if(jp2.left_bumper() || jp1.left_bumper()) robot.claw.toggleClawState(Claw.ClawSide.LEFT);
            if(jp2.right_bumper() || jp1.right_bumper()) robot.claw.toggleClawState(Claw.ClawSide.RIGHT);
            if(jp2.b() || jp1.b()) robot.claw.toggleClawState(Claw.ClawSide.BOTH);

            switch (state) {
                case -1:
                    robot.arm.setPivotTargetPos(0);
                    robot.arm.setExtensionTargetPos(500);
                    if(robot.arm.getExtensionCurrentPos() < 450) robot.wrist.setPosition(Constants.Wrist.INIT_POS);
                    else robot.wrist.setPosition(Constants.Wrist.INTAKE_POS);
                    break;
                case 0:
                    robot.arm.setPivotTargetPos(0);
                    robot.arm.setExtensionTargetPos(0);
                    if(robot.arm.getExtensionCurrentPos() > 50) robot.wrist.setPosition(Constants.Wrist.INIT_POS);
                    else robot.wrist.setPosition(Constants.Wrist.INTAKE_POS);
//                    robot.wrist.setPosition(Constants.Wrist.INTAKE_POS);
                    break;
                case 1:
                    robot.arm.setPivotTargetPos(150);
                    robot.arm.setExtensionTargetPos(0);
                    robot.wrist.setPosition(Constants.Wrist.INTAKE_POS);
                    break;
                case 2:
                    robot.arm.setPivotTargetPos(580);
                    robot.arm.setExtensionTargetPos(0);
                    robot.wrist.setPosition(Constants.Wrist.DEPO_POS_580);
                    break;
                case 3:
                    robot.arm.setPivotTargetPos(jp2.right_stick_y() * -.5 + robot.arm.getPivotTargetPos());
                    if(jp2.dpad_up() && robot.arm.getExtensionTargetPos() + ticksPerMajor <= maxExtension) robot.arm.setExtensionTargetPos(robot.arm.getExtensionTargetPos() + ticksPerMajor);
                    if(jp2.dpad_down() && robot.arm.getExtensionTargetPos() - ticksPerMajor >= 0) robot.arm.setExtensionTargetPos(robot.arm.getExtensionTargetPos() - ticksPerMajor);
                    if(jp2.dpad_right()) robot.arm.setPivotTargetPos(580);
//                    if(jp2.dpad_left() && robot.arm.getPivotTargetPos() - ticksPerMinor >= 0) robot.arm.setExtensionTargetPos(robot.arm.getPivotTargetPos() - ticksPerMinor);
                    if(jp2.dpad_left()) robot.arm.setPivotTargetPos(635);
                    break;
            }

            if(jp2.y()) state = Math.min(maxStates, state + 1);
            if(jp2.a()) state = Math.max(-1, state - 1);

            if(jp1.a()) isSlowMode = !isSlowMode;
            if(!isSlowMode) robot.drive.setPowers(jp1.left_stick_x(), -jp1.left_stick_y(), jp1.right_stick_x(), x -> .75 * Math.cbrt(x));
            else robot.drive.setPowers(jp1.left_stick_x(), -jp1.left_stick_y(), jp1.right_stick_x(), x -> Math.cbrt(x) * .4);

            if(jp1.guide()) robot.drone.launch();

            if(jp2.guide() && state == 0) {
                isHang = !isHang;
                justHung = isHang;
            }
            if(isHang) {
                if(jp2.left_stick_y() > 0) {
                    robot.hang.setPower(jp2.left_stick_y() * 1);
                }
                else {
                    robot.hang.setPower(jp2.left_stick_y() * .5);
                }
                robot.wrist.setPosition(Constants.Wrist.INIT_POS);
                robot.claw.setClawState(Claw.ClawSide.BOTH, Claw.ClawState.CLOSE);
            } else if(justHung) {
                robot.wrist.setPosition(Constants.Wrist.INTAKE_POS);
            }


            telemetry.addData("Current State", state);
            telemetry.addData("Slow Mode", isSlowMode);
            telemetry.addLine();
            telemetry.addData("Current Pivot Position", robot.arm.getPivotCurrentPos());
            telemetry.addData("Target Pivot Position", robot.arm.getPivotTargetPos());
            telemetry.addLine();
            telemetry.addData("Current Extension Position", robot.arm.getExtensionCurrentPos());
            telemetry.addData("Target Extension Position", robot.arm.getExtensionTargetPos());
            telemetry.addLine();
            telemetry.addData("Hang Powers", robot.hang.getPowers()[0] + " " + robot.hang.getPowers()[1]);


            telemetry.update();
            jp1.update();
            jp2.update();
            robot.update();
        }

    }
}
