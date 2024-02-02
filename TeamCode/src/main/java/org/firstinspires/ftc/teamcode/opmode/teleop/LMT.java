package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.photoncore.Photon;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.util.LoopRateTracker;
import org.firstinspires.ftc.teamcode.util.WPIMathUtil;
import org.firstinspires.ftc.teamcode.util.gamepad.JustPressed;

@Photon
@TeleOp(group = "game", name = "League Tournament Game Teleop")
public class LMT extends LinearOpMode {

    public static double DRIVETRAIN_POWER = 4;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, telemetry, Claw.ClawState.CLOSE);

        JustPressed gp1 = new JustPressed(gamepad1);
        JustPressed gp2 = new JustPressed(gamepad2);
        LoopRateTracker loopRateTracker = new LoopRateTracker();

//        robot.setArmState(Robot.ArmState.INIT);
        do {
            robot.update();
        } while (opModeInInit());
        waitForStart();
        robot.init();
        robot.setArmState(Robot.ArmState.INTAKE);
        robot.claw.setClawState(Claw.ClawSide.BOTH, Claw.ClawState.CLOSE);
        boolean isHang = false;
        while (opModeIsActive()) {
            robot.clearCache();
            robot.update();

            // drive
            robot.drive.setPowers(gp1.left_stick_x(), -gp1.left_stick_y(), gp1.right_stick_x(), x -> (Math.pow(x, DRIVETRAIN_POWER) * .75 * Math.signum(x)));

            // launch
            if(gp1.guide()) robot.launch.launch();

            // hang
            if(gp2.guide()) isHang = !isHang;
            if(isHang) {
                robot.setArmState(Robot.ArmState.INIT);
                robot.hang.setPower(gp2.left_stick_y() * .5);
            }

            //wrist


            // arm, claw, and wrist todo
            Robot.ArmState armState = robot.getArmState();
            if(armState == Robot.ArmState.DEPO) { // saucy control
                if(gp1.left_bumper()) robot.claw.setClawState(Claw.ClawSide.LEFT, robot.claw.getClawState(Claw.ClawSide.LEFT) == Claw.ClawState.CLOSE ? Claw.ClawState.OPEN : Claw.ClawState.CLOSE);
                if(gp1.right_bumper()) robot.claw.setClawState(Claw.ClawSide.RIGHT, robot.claw.getClawState(Claw.ClawSide.RIGHT) == Claw.ClawState.CLOSE ? Claw.ClawState.OPEN : Claw.ClawState.CLOSE);
                if(gp1.b()) robot.claw.setClawState(Claw.ClawSide.BOTH, robot.claw.getClawState(Claw.ClawSide.BOTH) == Claw.ClawState.CLOSE ? Claw.ClawState.OPEN : Claw.ClawState.CLOSE);

            }
            else if(armState != Robot.ArmState.TRANSITION || armState != Robot.ArmState.DEPO) { // ashwin control
                if(gp2.left_bumper()) robot.claw.setClawState(Claw.ClawSide.LEFT, robot.claw.getClawState(Claw.ClawSide.LEFT) == Claw.ClawState.CLOSE ? Claw.ClawState.OPEN : Claw.ClawState.CLOSE);
                if(gp2.right_bumper()) robot.claw.setClawState(Claw.ClawSide.RIGHT, robot.claw.getClawState(Claw.ClawSide.RIGHT) == Claw.ClawState.CLOSE ? Claw.ClawState.OPEN : Claw.ClawState.CLOSE);
                if(gp2.b()) robot.claw.setClawState(Claw.ClawSide.BOTH, robot.claw.getClawState(Claw.ClawSide.BOTH) == Claw.ClawState.CLOSE ? Claw.ClawState.OPEN : Claw.ClawState.CLOSE);
            }
//            if(gp2.dpad_up()) {
//                if(armState == Robot.ArmState.LEFT_INTAKE || armState == Robot.ArmState.BOTH_INTAKE || armState == Robot.ArmState.RIGHT_INTAKE) {
//                    robot.setArmState(Robot.ArmState.TRANSITION);
//                }
//                else if(armState == Robot.ArmState.TRANSITION) robot.setArmState(Robot.ArmState.DEPO);
//                else if(armState == Robot.ArmState.INIT) robot.setArmState(Robot.ArmState.BOTH_INTAKE);
//            }

            if(gp2.dpad_up()) {
                if(armState == Robot.ArmState.INTAKE) robot.setArmState(Robot.ArmState.TRANSITION);
                else if(armState == Robot.ArmState.TRANSITION) robot.setArmState(Robot.ArmState.DEPO);
            }
            if(gp2.dpad_down()) {
                if(armState == Robot.ArmState.DEPO) robot.setArmState(Robot.ArmState.TRANSITION);
                if(armState == Robot.ArmState.TRANSITION) {
                    robot.setArmState(Robot.ArmState.INTAKE);
                    robot.claw.setClawState(Claw.ClawSide.BOTH, Claw.ClawState.OPEN);
                }
            }
//            if(gp2.dpad_down()) {
//                if(armState == Robot.ArmState.DEPO) robot.setArmState(Robot.ArmState.TRANSITION);
//                if(armState == Robot.ArmState.TRANSITION) robot.setArmState(Robot.ArmState.BOTH_INTAKE);
//            }
            if(armState == Robot.ArmState.TRANSITION && gp2.b()) {
                robot.setArmState(Robot.ArmState.INTAKE);
                robot.claw.setClawState(Claw.ClawSide.BOTH, Claw.ClawState.OPEN);
            }
            if(gp2.left_bumper() && armState == Robot.ArmState.TRANSITION) {
                robot.setArmState(Robot.ArmState.INTAKE);
                robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.OPEN);
            }
            if(gp2.right_bumper() && armState == Robot.ArmState.TRANSITION){
                robot.setArmState(Robot.ArmState.INTAKE);
                robot.claw.setClawState(Claw.ClawSide.RIGHT, Claw.ClawState.OPEN);
            }

            if(armState == Robot.ArmState.DEPO) {
                if(gp2.left_stick_y() != 0)
                    robot.arm.setExtensionTargetPos(WPIMathUtil.clamp(robot.arm.getExtensionTargetPos() + gp2.left_stick_y() * 10, 0, 500));
                if(gp2.right_stick_y() != 0)
                    robot.arm.setPivotTargetPos(WPIMathUtil.clamp(robot.arm.getPivotTargetPos() + gp2.right_stick_y(), Arm.degreesToTicks(90), 580));
            }


            telemetry.addData("Arm State", armState);
            telemetry.addData("Pivot Current Pos", robot.arm.getPivotCurrentPos());
            telemetry.addData("Pivot Target Pos", robot.arm.getPivotTargetPos());

            telemetry.addData("Left Claw", robot.claw.getClawState(Claw.ClawSide.LEFT));
            telemetry.addData("Right Claw", robot.claw.getClawState(Claw.ClawSide.RIGHT));
            double[] clawPos = robot.claw.getClawPositions();
            telemetry.addLine("L: " + clawPos[0] + "; R: " + clawPos[1]);
            telemetry.addData("Loop Rate", loopRateTracker.getLoopTime());
            telemetry.addData("Hang Mode", isHang ? "ON" : "OFF");
            telemetry.addData("LEFT STICK", gp2.left_stick_y());
            telemetry.update();



            gp1.update();
            gp2.update();
            loopRateTracker.update();
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
