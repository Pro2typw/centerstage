package org.firstinspires.ftc.teamcode.opmode.autonomus;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.util.Constants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.TeamPropLocation;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.Vector;

@Autonomous(group = "0")
public class BlueStack0 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = TelemetryUtil.initTelemetry(telemetry);
        Robot robot = new Robot(hardwareMap, telemetry, Claw.ClawState.CLOSE);
        final Pose2d startingPose = new Pose2d(-36, 63, Math.toRadians(90));
        robot.drive.setPoseEstimate(startingPose);

        TeamPropLocation position = TeamPropLocation.LEFT;

        TrajectorySequence left = robot.drive.trajectorySequenceBuilder(startingPose)
                .setReversed(true)
                .lineTo(new Vector2d(-36, 51))
                .lineToLinearHeading(new Pose2d(-36, 33, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    // claw drop purple
                })
                .waitSeconds(1)
                .back(3)
                .addDisplacementMarker(() -> {
                    // pivot to stack for one
                })
                .lineToLinearHeading(new Pose2d(-59, 36, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    // claw pickup from stack
                })
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(-28, 60), 0)
                .addDisplacementMarker(() -> {
                    // pivot to depo
                })
                .lineTo(new Vector2d(12, 60))
                .splineTo(new Vector2d(48, 36), 0)
                .addDisplacementMarker(() -> {
                    // claw drop yellow
                })
                .build();

///RIGHT
        TrajectorySequence right = robot.drive.trajectorySequenceBuilder(startingPose)
                .setReversed(true)
                .lineTo(new Vector2d(-36, 51))
                .lineToLinearHeading(new Pose2d(-36, 33, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    // claw drop purple
                })
                .waitSeconds(1)
//                                        .back(3)
                .addDisplacementMarker(() -> {
                    // pivot to stack for one
                })
                .lineToLinearHeading(new Pose2d(-59, 36, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    // claw pickup from stack
                })
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(-28, 60), 0)
                .addDisplacementMarker(() -> {
                    // pivot to depo
                })
                .lineTo(new Vector2d(12, 60))
                .splineTo(new Vector2d(48, 36), 0)
                .addDisplacementMarker(() -> {
                    // claw drop yellow
                })
                .build();

//// CENTER
        TrajectorySequence center = robot.drive.trajectorySequenceBuilder(startingPose)
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    // point toward backdrop ready to drop
                    robot.wrist.setPosition(Constants.Wrist.INTAKE_POS);
                })
                .lineTo(new Vector2d(-42, 51))
                .lineToLinearHeading(new Pose2d(-34, 42, Math.toRadians(270)))
                .addDisplacementMarker(() -> {
                    robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.OPEN);

//                    robot.arm.setPivotTargetPos(30);
                    // claw drop purple
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(-36.1, 36))
                .addDisplacementMarker(() -> {
                    robot.arm.setPivotTargetPos(100);
                })
                .waitSeconds(1)
//                .lineTo(new Vector2d(-36.2, 36))
//                .addDisplacementMarker(() -> {
//
//                })
                .back(3)
                .addDisplacementMarker(() -> {
                    // claw pickup from stack
                    robot.arm.setPivotTargetPos(50);
//                    robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.CLOSE);
                })
                .lineToLinearHeading(new Pose2d(-52, 39.5, Math.toRadians(180)))
                .waitSeconds(.5)
                .lineToLinearHeading(new Pose2d(-58.5, 39.5, Math.toRadians(180)))
//                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.CLOSE);
                })
                .waitSeconds(2)
//                .lineTo(new Vector2d(-58.001, 40))
                .lineTo(new Vector2d(15, 40))
                .addDisplacementMarker(() -> {
                    // pivot to depo
                    robot.arm.setPivotTargetPos(580);
                    robot.wrist.setPosition(Constants.Wrist.DEPO_POS);
                })
                .waitSeconds(2)
                .lineTo(new Vector2d(46, 41))
                .addDisplacementMarker(() -> {
                    // claw drop yellow
                    robot.claw.setClawState(Claw.ClawSide.BOTH, Claw.ClawState.OPEN);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(43.9, 40))
                .waitSeconds(2)
                .addDisplacementMarker(() -> {
                    robot.arm.setPivotTargetPos(150);
                    robot.wrist.setPosition(Constants.Wrist.INTAKE_POS);
                })
                .build();


        do {
            switch (position) {
                case LEFT:
                    position = TeamPropLocation.RIGHT;
                    break;
                case RIGHT:
                    position = TeamPropLocation.CENTER;
                    break;
                case CENTER:
                    position = TeamPropLocation.LEFT;
                    break;
            }

            position = TeamPropLocation.CENTER;

            telemetry.addLine("Purple is on left claw");
            telemetry.addLine("Yellow is on right claw");
            telemetry.addData("Team Prop Location", position);
            telemetry.update();


            sleep(1000);
        }
        while (opModeInInit());

        waitForStart();
        robot.init();
        switch (position) {
            case LEFT:
                robot.drive.followTrajectorySequenceAsync(left);
                break;
            case RIGHT:
                robot.drive.followTrajectorySequenceAsync(right);
                break;
            case CENTER:
                robot.drive.followTrajectorySequenceAsync(center);
                break;
        }

        while (opModeIsActive()) {
            robot.update();
        }

    }
}
