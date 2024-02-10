package org.firstinspires.ftc.teamcode.opmode.autonomus;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

@Autonomous
public class Tes extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = TelemetryUtil.initTelemetry(telemetry);
        Robot robot = new Robot(hardwareMap, telemetry, Claw.ClawState.CLOSE);
        final Pose2d sp = new Pose2d(12, 63, Math.toRadians(90));
        TrajectorySequence seq = robot.drive.trajectorySequenceBuilder(sp)
                .setReversed(true)
                .lineTo(new Vector2d(12, 51))
                .lineToLinearHeading(new Pose2d(12, 33, Math.toRadians(270)))
                .addDisplacementMarker(() -> {
                    // claw drop purple
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(36, 36))
                .addDisplacementMarker(() -> {
                    // pivot to depo
                })
                .lineToLinearHeading(new Pose2d(50, 36, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    // claw drop yellow
                })
                .waitSeconds(1)
                .build();
        robot.drive.setPoseEstimate(sp);
        waitForStart();

        robot.drive.followTrajectorySequence(seq);
    }
}
