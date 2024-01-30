package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.util.gamepad.JustPressed;

@TeleOp(group = "game", name = "League Tournament Game Teleop")
public class LMT extends LinearOpMode {

    public static double DRIVETRAIN_POWER = 4;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry, Claw.ClawState.CLOSE);

        JustPressed gp1 = new JustPressed(gamepad1);
        JustPressed gp2 = new JustPressed(gamepad2);


        do {
            robot.update();
        } while (opModeInInit());
        waitForStart();
        while (opModeIsActive()) {
            robot.clearCache();
            robot.update();

            // drive
            robot.drive.setPowers(gp1.left_stick_x(), -gp1.left_stick_y(), gp1.right_stick_x(), x -> (Math.pow(x, DRIVETRAIN_POWER) * .75 * Math.signum(x)));

            // drone
            if(gp1.guide()) robot.launch.launch();

            // claw
//            if(gp2.left_bumper() || gp2.left_bumper()) robot.claw.setClawState(Claw.ClawSide.LEFT, robot.claw.getClawState(Claw.ClawSide.LEFT) == Claw.ClawState.CLOSE ? Claw.ClawState.OPEN : Claw.ClawState.CLOSE);
//            if(gp2.right_bumper() || gp2.right_bumper()) robot.claw.setClawState(Claw.ClawSide.RIGHT, robot.claw.getClawState(Claw.ClawSide.RIGHT) == Claw.ClawState.CLOSE ? Claw.ClawState.OPEN : Claw.ClawState.CLOSE);
//            if(gp2.b()) robot.claw.toggleClawState();
//            if(robot.drive.getPoseEstimate().getX() > 0) robot.claw.setFlipped(true);

            // hang
//            if(gp2.start()) robot.hang.cycleNextHangState();

            //wrist

            // arm
            if(Arm.ticksToDegrees(robot.arm.getCurrentDifferencePosition()) > 90) {
                double currentYComponent = robot.arm.getCurrentAveragePosition() * Math.sin(Math.toRadians(Arm.ticksToDegrees(robot.arm.getCurrentDifferencePosition())));
                double targetXComp = Arm.backdropYtoX(currentYComponent);
                robot.arm.setExtensionTargetPos(targetXComp / Math.cos(Math.toRadians(Arm.ticksToDegrees(robot.arm.getCurrentDifferencePosition()))));
            }

            //launch (switch up the holding (back or mode button) and x-clicking based on preference [it should be ashwin clicking x btw yall r dumb] )
            if(gamepad1.guide)
                if(gamepad2.x)
                    robot.launch.launch();



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
