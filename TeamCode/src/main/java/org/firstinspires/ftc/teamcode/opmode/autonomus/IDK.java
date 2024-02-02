package org.firstinspires.ftc.teamcode.opmode.autonomus;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "test")
public class IDK extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap);

        final Pose2d StartingPose = new Pose2d(new Vector2d(-40, 62), Math.toRadians(90));

        TrajectorySequence seq = drive.trajectorySequenceBuilder(StartingPose)
                .setReversed(true)
                .splineTo(new Vector2d(-36, 35), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    // Place preloaded pixel on tape
                })
                .waitSeconds(1)
                .setReversed(false)
                .splineTo(new Vector2d(-52, 35), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    // Grab pixel from stack
                })
                .waitSeconds(1)

                .lineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(180)))
                .lineTo(new Vector2d(11,58))
                .splineTo(new Vector2d(48, 34), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    // Place pixel on backboard
                })
                .waitSeconds(1)
                .setReversed(false)
                .splineTo(new Vector2d(11, 58), Math.toRadians(180))
                .lineTo(new Vector2d(-32,58))
                .lineToSplineHeading(new Pose2d(-52, 35, Math.toRadians(180)))

// Cycle 2:
                .addDisplacementMarker(() -> {
                    // Grab pixel from stack
                })
                .waitSeconds(1)
                .lineToSplineHeading(new Pose2d(-32, 58, Math.toRadians(180)))
                .lineTo(new Vector2d(11,58))
                .splineTo(new Vector2d(48, 34), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    // Place pixel on backboard
                })
                .waitSeconds(1)
// Park:
                .build();

        waitForStart();
        drive.setPoseEstimate(StartingPose);
        drive.followTrajectorySequence(seq);

        while (opModeIsActive()) {
            drive.update();


        }
    }
}
