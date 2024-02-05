package org.firstinspires.ftc.teamcode.opmode.autonomus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.TeamPropLocation;

import java.util.HashMap;

@Autonomous(group = "game", name = "RedStack 2+0")
public class RedStack extends LinearOpMode {




    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, telemetry, Claw.ClawState.CLOSE);
        HashMap<TeamPropLocation, TrajectorySequence> paths = new HashMap<>();

        TeamPropLocation location = TeamPropLocation.LEFT;

        final Pose2d startPose = new Pose2d(12, -63, Math.toRadians(270));
        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence left = robot.drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    // point toward backdrop ready to drop
                    robot.setArmState(Robot.ArmState.INTAKE);
                })
                .lineTo(new Vector2d(-36, -51))
                .lineToLinearHeading(new Pose2d(-36, -33, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    // claw drop purple
                    robot.claw.setClawState(Claw.ClawSide.RIGHT, Claw.ClawState.OPEN);
                })
                .waitSeconds(1)
//                                        .back(3)
                .addDisplacementMarker(() -> {
                    // pivot to stack for one
                })
                .lineToLinearHeading(new Pose2d(-59, -36, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    // claw pickup from stack
                })
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(-28, -60), 0)
                .addDisplacementMarker(() -> {
                    // pivot to depo
                })
                .lineTo(new Vector2d(12, -60))
                .splineTo(new Vector2d(48, -36), 0)
                .addDisplacementMarker(() -> {
                    // claw drop yellow
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence right = robot.drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    // point toward backdrop ready to drop
                    robot.setArmState(Robot.ArmState.INTAKE);
                })
                .lineTo(new Vector2d(-36, -51))
                .lineToLinearHeading(new Pose2d(-32, -33, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    // claw drop purple
                })
                .waitSeconds(1)
                .back(3)
                .addDisplacementMarker(() -> {
                    // pivot to stack for one
                })
                .lineToLinearHeading(new Pose2d(-59, -36, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    // claw pickup from stack
                })
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(-28, -60), 0)
                .addDisplacementMarker(() -> {
                    // pivot to depo
                })
                .lineTo(new Vector2d(12, -60))
                .splineTo(new Vector2d(48, -36), 0)
                .addDisplacementMarker(() -> {
                    // claw drop yellow
                })
                .build();

        TrajectorySequence center = robot.drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    // point toward backdrop ready to drop
                    robot.setArmState(Robot.ArmState.INTAKE);
                })
                .lineTo(new Vector2d(-36, -51))
                .lineToLinearHeading(new Pose2d(-36, -33, Math.toRadians(270)))
                .addDisplacementMarker(() -> {
                    // claw drop purple
                })
                .waitSeconds(1)
                .forward(3)
                .addDisplacementMarker(() -> {
                    // pivot to stack for one
                })
                .lineToLinearHeading(new Pose2d(-59, -36, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    // claw pickup from stack
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(0, -36))
                .addDisplacementMarker(() -> {
                    // pivot to depo
                })
                .lineTo(new Vector2d(50, -36))
                .addDisplacementMarker(() -> {
                    // claw drop yellow
                })
                .waitSeconds(1)
                .build();

        paths.put(TeamPropLocation.LEFT, left);
        paths.put(TeamPropLocation.RIGHT, right);
        paths.put(TeamPropLocation.CENTER, center);


        while(opModeInInit()) {
            if(location == TeamPropLocation.LEFT) location = TeamPropLocation.CENTER;
            else if(location == TeamPropLocation.CENTER) location = TeamPropLocation.RIGHT;
            else location = TeamPropLocation.LEFT;

            telemetry.addData("Location", location);
            telemetry.update();

            sleep(1000);
        }

        waitForStart();
        robot.init();


        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        location = TeamPropLocation.RIGHT;

        TrajectorySequence path = paths.get(location);
        int i = 0;
        while (opModeIsActive()) {
            if(i++ < 1 && !robot.drive.isBusy()) robot.drive.followTrajectorySequenceAsync(path);


            telemetry.addLine("Left is Yellow, Right is Purple");
            telemetry.addData("Location", location);
            telemetry.update();
            robot.update();

        }

    }
}
