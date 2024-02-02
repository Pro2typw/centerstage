package org.firstinspires.ftc.teamcode.opmode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.util.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystem.util.AllianceSide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.TeamPropLocation;
import org.jetbrains.annotations.NotNull;

import java.util.HashMap;

public class TrajectorySequencer {
    public static HashMap<TeamPropLocation[], TrajectorySequence> getTrajectorSequence(@NotNull Robot robot, @NotNull AllianceColor color, @NotNull AllianceSide side, int cycles) {
        HashMap<TeamPropLocation[], TrajectorySequence> hashmap = new HashMap<>(3);

        Pose2d startPose;
        if(color == AllianceColor.BLUE) startPose = side == AllianceSide.BACKDROP ? new Pose2d(12, 63, Math.toRadians(90)) : new Pose2d();
        else startPose = side == AllianceSide.BACKDROP ? new Pose2d(12, 63 * -1, Math.toRadians(270)) : new Pose2d();

        // add trajs...
        TrajectorySequence left, right, center;

        if(color == AllianceColor.BLUE) {
            if(side == AllianceSide.BACKDROP) {
                left = robot.drive.trajectorySequenceBuilder(startPose)
                        .setReversed(true)
                        .addDisplacementMarker(() -> {
                            // point toward backdrop ready to drop
                        })
                        .splineTo(new Vector2d(32, 36), Math.toRadians(0))
                        .addDisplacementMarker(() -> {
                            // drop purple pixel
                        })
                        .waitSeconds(1)
                        .addDisplacementMarker(() -> {
                            // pivot to backdrop with wrist
                        })
                        .lineTo(new Vector2d(50, 36))
                        .addDisplacementMarker(() -> {
                            // drop yellow pixel on backdrop
                            // rotate extension and pivot to match purple
                            // drop purple pixel
                        })
                        .waitSeconds(1)
                        .build();

                right = robot.drive.trajectorySequenceBuilder(startPose)
                        .setReversed(true)
                        .addDisplacementMarker(() -> {
                            // point toward backdrop ready to drop
                        })
                        .splineTo(new Vector2d(32, 36), Math.toRadians(0))
                        .lineTo(new Vector2d(10, 36))
                        .addDisplacementMarker(() -> {
                            // drop purple pixel
                        })
                        .waitSeconds(1)
                        .addDisplacementMarker(() -> {
                            // pivot to backdrop with wrist
                        })
                        .lineTo(new Vector2d(50, 36))
                        .addDisplacementMarker(() -> {
                            // drop yellow pixel on backdrop
                            // rotate extension and pivot to match purple
                            // drop purple pixel
                        })
                        .waitSeconds(1)
                        .build();

                center = robot.drive.trajectorySequenceBuilder(startPose)
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
            }
            else {
                left = robot.drive.trajectorySequenceBuilder(startPose)
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

                right = robot.drive.trajectorySequenceBuilder(startPose)
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
                        .waitSeconds(1)
                        .build();

                center = robot.drive.trajectorySequenceBuilder(startPose)
                        .setReversed(true)
                        .lineTo(new Vector2d(-36, 51))
                        .lineToLinearHeading(new Pose2d(-36, 33, Math.toRadians(270)))
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
                        .lineTo(new Vector2d(0, 36))
                        .addDisplacementMarker(() -> {
                            // pivot to depo
                        })
                        .lineTo(new Vector2d(50, 36))
                        .addDisplacementMarker(() -> {
                            // claw drop yellow
                        })
                        .waitSeconds(1)
                        .build();
            }
        }

        TrajectorySequence cycle = robot.drive.trajectorySequenceBuilder(startPose)
                .build();


        return hashmap;
    }
}
