package org.firstinspires.ftc.teamcode.opmode.autonomus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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

@Config
@Autonomous(group = "game", name = "TD")
public class TDOP extends LinearOpMode {

    public static double move = 22;
    public static double move1 = 18;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, telemetry, Claw.ClawState.CLOSE);
        HashMap<TeamPropLocation, TrajectorySequence> paths = new HashMap<>();

        TeamPropLocation location = TeamPropLocation.LEFT;

        final Pose2d startPose = new Pose2d(12, -63, Math.toRadians(270));
        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence park = robot.drive.trajectorySequenceBuilder(startPose)
                .back(move)
                .forward(move1)
                .build();


        paths.put(TeamPropLocation.LEFT, park);

        waitForStart();
        robot.init();


        robot.drive.followTrajectorySequenceAsync(park);

        int i = 0;
        while (opModeIsActive()) {

            telemetry.addLine("Left is Yellow, Right is Purple");
            telemetry.addData("Location", location);
            telemetry.update();
            robot.update();

        }

    }
}
