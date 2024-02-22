package org.firstinspires.ftc.teamcode.opmode.game.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rr.commands.ArmC;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

public class BlueBackdrop0 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, telemetry, Claw.ClawState.OPEN);
        ArmC armC = new ArmC(robot);

        Action[] left = new Action[] {};
        Action[] center = new Action[] {};
        Action[] right = new Action[] {};

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        left[0], // turn to purple
                                        armC.setWristPosition(Constants.Wrist.INTAKE_POS)
                                ),
                                armC.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.OPEN),
                                new ParallelAction(
                                        armC.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.CLOSE),
                                        left[1], // move to badckdrop
                                        armC.toDeposit()
                                ),
                                armC.setClawState(Claw.ClawSide.RIGHT, Claw.ClawState.OPEN),
                                armC.waitSeconds(500)
                                ),
                        armC.update()
                )
        );


    }
}
