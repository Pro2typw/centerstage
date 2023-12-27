package org.firstinspires.ftc.teamcode.opmode.util;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.TeamPropLocation;

import java.util.HashMap;

public class TrajectorySequencer {
    public static HashMap<TeamPropLocation, TrajectorySequence> getTrajectorSequence(MecanumDrive drive, AllianceColor color, AllianceSide side) {
        HashMap<TeamPropLocation, TrajectorySequence> hashmap = new HashMap<>(3);

        double colorMultiplier = color == AllianceColor.BLUE ? 1 : -1;
        double sideMultiplier = side == AllianceSide.BACKDROP ? 1 : -1;


        return hashmap;
    }
}
