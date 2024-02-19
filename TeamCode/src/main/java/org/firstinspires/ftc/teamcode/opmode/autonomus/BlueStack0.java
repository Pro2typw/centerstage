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
        final Pose2d startingPose = new Pose2d(-39, 63, Math.toRadians(90));
        robot.drive.setPoseEstimate(startingPose);

        TeamPropLocation position = TeamPropLocation.LEFT;

////LEFT
        TrajectorySequence left = robot.drive.trajectorySequenceBuilder(startingPose)
                .setReversed(true)
                .lineTo(new Vector2d(-36, 51))
                .lineToLinearHeading(new Pose2d(-36, 33, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    // claw drop purple
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(-36, 60))

                .setReversed(true)
                .lineTo(new Vector2d(12, 60))
                .splineTo(new Vector2d(48, 36), 0)
                .addDisplacementMarker(() -> {
                    // pivot and drop yellow pixel on left backboard
                    // park (just stay in front of backboard)
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
                .lineTo(new Vector2d(-36, 60))

                .setReversed(true)
                .lineTo(new Vector2d(12, 60))
                .splineTo(new Vector2d(48, 36), 0)
                .addDisplacementMarker(() -> {
                    // pivot and drop yellow pixel on left backboard
                    // park (just stay in front of backboard)
                })
                .build();

//// CENTER
        TrajectorySequence center = robot.drive.trajectorySequenceBuilder(startingPose)
                .setReversed(true)
                .lineTo(new Vector2d(-36, 51))
                .lineToLinearHeading(new Pose2d(-36, 35, Math.toRadians(270)))
                .addDisplacementMarker(() -> {
                    // claw drop purple
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-36, 60, Math.toRadians(180)))

                .setReversed(true)
                .lineTo(new Vector2d(12, 60))
                .splineTo(new Vector2d(48, 36), 0)
                .addDisplacementMarker(() -> {
                    // pivot and drop yellow pixel on left backboard
                    // park (just stay in front of backboard)
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
