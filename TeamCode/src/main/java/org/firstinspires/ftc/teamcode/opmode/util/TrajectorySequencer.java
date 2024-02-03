package org.firstinspires.ftc.teamcode.opmode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.util.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystem.util.AllianceSide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.TeamPropLocation;
import org.jetbrains.annotations.NotNull;

import java.util.HashMap;

public class TrajectorySequencer {



    /**
     * Returns a hashmap of TrajectorySequences for each location and cycle
     * @param robot the robot
     * @param color the alliance color
     * @param side the alliance side
     * @param cycles the number of cycles
     * @return a hashmap of TrajectorySequences for each location and cycle
     */
    public static HashMap<TeamPropLocation, TrajectorySequence[]> getTrajectorSequence(@NotNull Robot robot, @NotNull AllianceColor color, @NotNull AllianceSide side, int cycles) {
        HashMap<TeamPropLocation, TrajectorySequence[]> hashmap = new HashMap<>(3);

        final Pose2d startPose;
        if(color == AllianceColor.BLUE) startPose = side == AllianceSide.BACKDROP ? new Pose2d(12, 63, Math.toRadians(90)) : new Pose2d();
        else startPose = side == AllianceSide.BACKDROP ? new Pose2d(12, 63 * -1, Math.toRadians(270)) : new Pose2d();

        TrajectorySequence left, right, center;

        if(color == AllianceColor.BLUE) {
            if(side == AllianceSide.BACKDROP) {
                left = robot.drive.trajectorySequenceBuilder(startPose)
                        .setReversed(true)
                        .addDisplacementMarker(() -> {
                            // point toward backdrop ready to drop
                            robot.setArmState(Robot.ArmState.INTAKE);
                        })
                        .splineTo(new Vector2d(40, 36), Math.toRadians(0))
                        .addDisplacementMarker(() -> {
                            // drop purple pixel
                            robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.OPEN);
                        })
                        .waitSeconds(1)
                        .lineTo(new Vector2d(41, 36))

                        .addDisplacementMarker(() -> {
                            // pivot to backdrop with wrist
                            robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.CLOSE);
                            robot.setArmState(Robot.ArmState.TRANSITION);

                        })
                        .lineTo(new Vector2d(42, 36))
                        .waitSeconds(6)
                        .addDisplacementMarker(() -> {
                            robot.setArmState(Robot.ArmState.DEPO);
                        })
                        .lineTo(new Vector2d(55, 52))
                        .waitSeconds(2)
                        .addDisplacementMarker(() -> {
                            // drop yellow pixel on backdrop
                            robot.claw.setClawState(Claw.ClawSide.RIGHT, Claw.ClawState.OPEN);
                        })
                        .waitSeconds(1)
                        .build();

                right = robot.drive.trajectorySequenceBuilder(startPose)
                        .setReversed(true)
                        .addDisplacementMarker(() -> {
                            // point toward backdrop ready to drop
                            robot.setArmState(Robot.ArmState.INTAKE);
                        })
                        .splineTo(new Vector2d(32, 36), Math.toRadians(0))
                        .lineTo(new Vector2d(12, 36))
                        .addDisplacementMarker(() -> {
                            // drop purple pixel
                            robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.OPEN);
                        })
                        .waitSeconds(1)
                        .lineTo(new Vector2d(14, 36))
                        .addDisplacementMarker(() -> {
                            robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.CLOSE);
                            robot.setArmState(Robot.ArmState.TRANSITION);
                        })
                        .lineTo(new Vector2d(16, 36))
                        .waitSeconds(6)
                        .addDisplacementMarker(() -> {
                            robot.setArmState(Robot.ArmState.DEPO);
                        })
                        .lineTo(new Vector2d(55, 36))
                        .addDisplacementMarker(() -> {
                            // drop yellow pixel on backdrop
                            robot.claw.setClawState(Claw.ClawSide.RIGHT, Claw.ClawState.OPEN);
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

        else { // todo
            if(side == AllianceSide.BACKDROP) {
                left = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
                center = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
                right = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();

            }
            else {
                left = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
                center = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
                right = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();

            }
        }


        TrajectorySequence cycleSide;
        TrajectorySequence cycleCenter;

        if(color == AllianceColor.BLUE) {
            cycleSide = robot.drive.trajectorySequenceBuilder(startPose)
                    .setTangent(Math.toRadians(180))
                    .addDisplacementMarker(() -> {
                        // pivot to intake
                    })
                    .splineTo(new Vector2d(20, 12), Math.toRadians(180))
//                                        .setTangent(Math.toRadians(180))
                    .lineTo(new Vector2d(-60, 12))
                    .build();
            cycleCenter = robot.drive.trajectorySequenceBuilder(startPose)
                    .addDisplacementMarker(() -> {
                        // pivot to intake
                    })
                    .lineTo(new Vector2d(-60, 36))
                    .addDisplacementMarker(() -> {
                        // claw close
                    })
                    .waitSeconds(1)
                    .lineTo(new Vector2d(12, 36))
                    .addDisplacementMarker(() -> {
                        // depo position
                    })
                    .lineTo(new Vector2d(50, 36))
                    .addDisplacementMarker(() -> {
                        // claw drop yellow
                    })
                    .waitSeconds(1)
                    .build();
        }
        else { // todo
            cycleSide = robot.drive.trajectorySequenceBuilder(startPose)

                    .build();
            cycleCenter = robot.drive.trajectorySequenceBuilder(startPose)
                    .build();
        }

        hashmap.put(TeamPropLocation.LEFT, new TrajectorySequence[1 + cycles]);
        hashmap.put(TeamPropLocation.RIGHT, new TrajectorySequence[1 + cycles]);
        hashmap.put(TeamPropLocation.CENTER, new TrajectorySequence[1 + cycles]);

        hashmap.get(TeamPropLocation.LEFT)[0] = left;
        hashmap.get(TeamPropLocation.RIGHT)[0] = right;
        hashmap.get(TeamPropLocation.CENTER)[0] = center;

        for(int i = 1; i <= cycles; i++) {
            hashmap.get(TeamPropLocation.LEFT)[i] = cycleSide;
            hashmap.get(TeamPropLocation.CENTER)[i] = cycleCenter;
            hashmap.get(TeamPropLocation.RIGHT)[i] = cycleSide;
        }

        return hashmap;
    }
}
