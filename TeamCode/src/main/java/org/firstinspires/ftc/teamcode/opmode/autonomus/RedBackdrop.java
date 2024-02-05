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

@Autonomous(group = "game", name = "RedBackdrop 2+0")
public class RedBackdrop extends LinearOpMode {




    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, telemetry, Claw.ClawState.CLOSE);
        HashMap<TeamPropLocation, TrajectorySequence> paths = new HashMap<>();

        TeamPropLocation location = TeamPropLocation.LEFT;

        final Pose2d startPose = new Pose2d(12, -63, Math.toRadians(270));
        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence left = robot.drive.trajectorySequenceBuilder(startPose)
//                .setReversed(true)
                .addDisplacementMarker(() -> {
                    // point toward backdrop ready to drop
                    robot.setArmState(Robot.ArmState.INTAKE);
                })
                .splineTo(new Vector2d(40, -36), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    // drop purple pixel
                    robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.OPEN);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(41, -36))

                .addDisplacementMarker(() -> {
                    // pivot to backdrop with wrist
                    robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.CLOSE);
                    robot.setArmState(Robot.ArmState.TRANSITION);

                })
                .lineTo(new Vector2d(42, -36))
                .waitSeconds(6)
                .addDisplacementMarker(() -> {
                    robot.setArmState(Robot.ArmState.DEPO);
                })
                .lineTo(new Vector2d(55, -52))
                .waitSeconds(2)
                .addDisplacementMarker(() -> {
                    // drop yellow pixel on backdrop
                    robot.claw.setClawState(Claw.ClawSide.RIGHT, Claw.ClawState.OPEN);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(54.5, -52))
                .addDisplacementMarker(() -> {
                    robot.setArmState(Robot.ArmState.TRANSITION);
                })
                .waitSeconds(2)
                .lineTo(new Vector2d(54.4, -52))
                .addDisplacementMarker(() -> {
                    robot.setArmState(Robot.ArmState.INTAKE);
                })
                .build();

        TrajectorySequence right = robot.drive.trajectorySequenceBuilder(startPose)
//                .setReversed(true)
                .addDisplacementMarker(() -> {
                    // point toward backdrop ready to drop
                    robot.setArmState(Robot.ArmState.INTAKE);
                })
                .splineTo(new Vector2d(32, -36), Math.toRadians(0))
                .lineTo(new Vector2d(12, -36))
                .addDisplacementMarker(() -> {
                    // drop purple pixel
                    robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.OPEN);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(14, -36))
                .addDisplacementMarker(() -> {
                    robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.CLOSE);
                    robot.setArmState(Robot.ArmState.TRANSITION);
                })
                .lineTo(new Vector2d(16, -36))
                .waitSeconds(6)
                .addDisplacementMarker(() -> {
                    robot.setArmState(Robot.ArmState.DEPO);
                })
                .lineTo(new Vector2d(55, -36))
                .addDisplacementMarker(() -> {
                    // drop yellow pixel on backdrop
                    robot.claw.setClawState(Claw.ClawSide.RIGHT, Claw.ClawState.OPEN);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(54.5, -36))
                .addDisplacementMarker(() -> {
                    robot.setArmState(Robot.ArmState.TRANSITION);
                })
                .waitSeconds(2)
                .lineTo(new Vector2d(54.4, -36))
                .addDisplacementMarker(() -> {
                    robot.setArmState(Robot.ArmState.INTAKE);
                })
                .build();

        TrajectorySequence center = robot.drive.trajectorySequenceBuilder(startPose)
//                .setReversed(true)
                .addDisplacementMarker(() -> {
                    // point toward backdrop ready to drop
                    robot.setArmState(Robot.ArmState.INTAKE);
                })
                .lineTo(new Vector2d(12, -51))
                .lineToLinearHeading(new Pose2d(12, -60, Math.toRadians(270)))
                .addDisplacementMarker(() -> {
                    // claw drop purple
                    robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.OPEN);

                })
                .waitSeconds(1)
                .lineTo(new Vector2d(14, -46))
                .addDisplacementMarker(() -> {
                    robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.CLOSE);
                    robot.setArmState(Robot.ArmState.TRANSITION);
                })
                .lineTo(new Vector2d(16, -46))
                .waitSeconds(6)
                .addDisplacementMarker(() -> {
                    robot.setArmState(Robot.ArmState.DEPO);
                })
                .lineTo(new Vector2d(36, -46))
                .addDisplacementMarker(() -> {
                    // pivot to depo
                    robot.setArmState(Robot.ArmState.DEPO);
                })
                .lineToLinearHeading(new Pose2d(55, -45, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    // claw drop yellow
                    robot.claw.setClawState(Claw.ClawSide.RIGHT, Claw.ClawState.OPEN);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(54.5, -45))
                .addDisplacementMarker(() -> {
                    robot.setArmState(Robot.ArmState.TRANSITION);
                })
                .waitSeconds(2)
                .lineTo(new Vector2d(54.4, -45))
                .addDisplacementMarker(() -> {
                    robot.setArmState(Robot.ArmState.INTAKE);
                })
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
