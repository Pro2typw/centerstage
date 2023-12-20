package org.firstinspires.ftc.teamcode.vision.test.pipeline;


import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.gamepad.JustPressed;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortalImpl;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class AprilTagDetectionPipeline extends LinearOpMode {

    AprilTagProcessor aprilTagProcessor;
    VisionPortal portal;
    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));

        MultipleTelemetry dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        JustPressed jp1 = new JustPressed(gamepad1);
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTagProcessor)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();

        portal.setProcessorEnabled(aprilTagProcessor, true);



        while (opModeInInit()) {
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            telemetry.addData("a", detections.size());

            if(detections.size() != 0) {
                AprilTagDetection detection = detections.get(0);
                telemetry.addData("heading", detection.ftcPose.yaw);
                if(gamepad1.a && !drive.isBusy()) {
                    drive.turn(Math.toRadians(detection.ftcPose.yaw * .1));
                }
            }

            telemetry.update();
            drive.update();
        }

        waitForStart();
    }
}