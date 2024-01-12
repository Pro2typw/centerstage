package org.firstinspires.ftc.teamcode.opmode.util;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.util.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystem.util.AllianceSide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.TeamPropLocation;

import java.util.HashMap;

public class TrajectorySequencer {
    public static HashMap<TeamPropLocation, TrajectorySequence> getTrajectorSequence(MecanumDrive drive, AllianceColor color, AllianceSide side) {
        HashMap<TeamPropLocation, TrajectorySequence> hashmap = new HashMap<>(3);

        double colorMultiplier = color == AllianceColor.BLUE ? 1 : -1;
        double sideMultiplier = side == AllianceSide.BACKDROP ? 1 : -1;

        // add trajs...

        return hashmap;
    }
}
