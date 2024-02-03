package org.firstinspires.ftc.teamcode.opmode.autonomus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmode.util.TrajectorySequencer;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.util.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystem.util.AllianceSide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.TeamPropLocation;

import java.util.HashMap;

@Autonomous(group = "game", name = "BlueBackdrop 2+0")
public class BlueBackdrop extends LinearOpMode {




    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, telemetry, Claw.ClawState.CLOSE);
        HashMap<TeamPropLocation, TrajectorySequence[]> paths = TrajectorySequencer.getTrajectorSequence(robot, AllianceColor.BLUE, AllianceSide.BACKDROP, 0);

        TeamPropLocation location = TeamPropLocation.LEFT;

        robot.drive.setPoseEstimate(new Pose2d(12, 63, Math.toRadians(90)));
//        Trajectory traj = robot.drive.trajectoryBuilder(new Pose2d())
//                .splineTo(new Vector2d(30, 30), 0)
//                .build();

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


//        robot.drive.followTrajectoryAsync(traj);
        for(TrajectorySequence seq : paths.get(location)) {
            robot.drive.followTrajectorySequence(seq);
        }

        while (opModeIsActive()) {
            robot.clearCache();
            robot.update();

        }

    }
}
