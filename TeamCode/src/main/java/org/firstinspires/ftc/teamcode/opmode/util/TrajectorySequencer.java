package org.firstinspires.ftc.teamcode.opmode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

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
     * @return a hashmap of TrajectorySequences for each location
     */
    public static HashMap<TeamPropLocation, TrajectorySequence[]> getTrajectorSequence(@NotNull Robot robot, @NotNull AllianceColor color, @NotNull AllianceSide side, int cycles) {
        HashMap<TeamPropLocation, TrajectorySequence[]> hashmap = new HashMap<>(3);

        Pose2d startPose;
        if(color == AllianceColor.BLUE) startPose = side == AllianceSide.BACKDROP ? new Pose2d(12, 63, Math.toRadians(90)) : new Pose2d();
        else startPose = side == AllianceSide.BACKDROP ? new Pose2d(12, -63, Math.toRadians(270)) : new Pose2d();

        TrajectorySequence left, right, center;

        if(color == AllianceColor.BLUE) {
            if(side == AllianceSide.BACKDROP) {
                left = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
                right = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
                center = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
            }
            else {
                left = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
                right = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
                center = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
            }
        }
        else {
            if(side == AllianceSide.BACKDROP) {
                left = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
                right = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
                center = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
            }
            else {
                left = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
                right = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
                center = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
            }
        }

        TrajectorySequence cycleSide;
        TrajectorySequence cycleCenter;

        startPose = new Pose2d();

        if(color == AllianceColor.BLUE) {
            if(side == AllianceSide.BACKDROP) {
                cycleSide = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
                cycleCenter = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
            }
            else {
                cycleSide = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
                cycleCenter = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
            }
        }
        else {
            if(side == AllianceSide.BACKDROP) {
                cycleSide = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
                cycleCenter = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
            }
            else {
                cycleSide = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
                cycleCenter = robot.drive.trajectorySequenceBuilder(startPose)
                        .build();
            }
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
