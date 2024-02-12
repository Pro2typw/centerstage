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

@Autonomous(group = "0")
public class RedBackdrop0 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = TelemetryUtil.initTelemetry(telemetry);
        Robot robot = new Robot(hardwareMap, telemetry, Claw.ClawState.CLOSE);
        final Pose2d startingPose = new Pose2d(12, -63, Math.toRadians(270));
        robot.drive.setPoseEstimate(startingPose);

        TeamPropLocation position = TeamPropLocation.LEFT;

        TrajectorySequence left = robot.drive.trajectorySequenceBuilder(startingPose)
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    // point toward backdrop ready to drop
                                    robot.wrist.setPosition(Constants.Wrist.INTAKE_POS);
                })
                .splineTo(new Vector2d(32, -36), Math.toRadians(0))
                .lineTo(new Vector2d(10, -36))
                .addDisplacementMarker(() -> {
                    // drop purple pixel
                                    robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.OPEN);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(12, -36))
                .addDisplacementMarker(() -> {
                    // pivot to backdrop with wrist
                                    robot.arm.setPivotTargetPos(150);
                })
                .lineTo(new Vector2d(20, -36))
                .addDisplacementMarker(() -> {
                                    robot.arm.setPivotTargetPos(580);
                                    robot.wrist.setPosition(Constants.Wrist.DEPO_POS);
                                    robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.CLOSE);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(45, -32))
                .addDisplacementMarker(() -> {
                    // drop yellow pixel on backdrop
                    // rotate extension and pivot to match purple
                    // drop purple pixel
                                    robot.claw.setClawState(Claw.ClawSide.RIGHT, Claw.ClawState.OPEN);
                })
                .waitSeconds(1)

                .build();
        TrajectorySequence right = robot.drive.trajectorySequenceBuilder(startingPose)
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    // point toward backdrop ready to drop
                                    robot.wrist.setPosition(Constants.Wrist.INTAKE_POS);
                })
                .splineTo(new Vector2d(35, -36), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    // drop purple pixel
                                    robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.OPEN);
                })
                .lineTo(new Vector2d(35.1, -36))
                .waitSeconds(.25)
                .addDisplacementMarker(() -> {
                                    robot.arm.setPivotTargetPos(150);
                })
                .waitSeconds(.5)
                .lineTo(new Vector2d(37, -36))
                .addDisplacementMarker(() -> {
                    // pivot to backdrop with wrist
                                    robot.arm.setPivotTargetPos(580);
                                    robot.wrist.setPosition(Constants.Wrist.DEPO_POS);
                                    robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.CLOSE);
                })
                .lineTo(new Vector2d(40, -43))
                .waitSeconds(1)
                .lineTo(new Vector2d(45, -50))
                .addDisplacementMarker(() -> {
                    // drop yellow pixel on backdrop
                                    robot.claw.setClawState(Claw.ClawSide.RIGHT, Claw.ClawState.OPEN);
                    // rotate extension and pivot to match purple
                    // drop purple pixel
                })
                .build();
        TrajectorySequence center = robot.drive.trajectorySequenceBuilder(startingPose)
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    // point toward backdrop ready to drop
                                    robot.wrist.setPosition(Constants.Wrist.INTAKE_POS);
                })
                .lineTo(new Vector2d(15, -51))
                .lineToLinearHeading(new Pose2d(15, -33, Math.toRadians(270)))
                .addDisplacementMarker(() -> {
                    // claw drop purple
                                    robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.OPEN);
                })
                .waitSeconds(.5)
                .lineTo(new Vector2d(12.1, -36))
                .addDisplacementMarker(() -> {
                                    robot.arm.setPivotTargetPos(150);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(36, -36))
                .addDisplacementMarker(() -> {
                    // pivot to depo
//                                    robot.arm.setPivotTargetPos(580);
//                                    robot.wrist.setPosition(Constants.Wrist.DEPO_POS);
//                                    robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.CLOSE);
                })
                .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    // claw drop yellow
//                                    robot.claw.setClawState(Claw.ClawSide.RIGHT, Claw.ClawState.OPEN);
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
