package org.firstinspires.ftc.teamcode.vision.test;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.gamepad.JustPressed;
import org.firstinspires.ftc.teamcode.vision.pipeline.AprilTagDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(group = "test")
public class TurnToTag extends LinearOpMode {
    AprilTagDetectionPipeline processor;

    boolean isTurnToTag = false;
    @Override
    public void runOpMode() throws InterruptedException {
        processor = new AprilTagDetectionPipeline();
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(processor.getAprilTagProcessor())
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        JustPressed jp = new JustPressed(gamepad1);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("RUNNING");
            telemetry.addData("turn", isTurnToTag);
            if(jp.a()) isTurnToTag = !isTurnToTag;
            List<AprilTagDetection> detections = processor.getAprilTagDetections();
            if(detections.size() >0) {
                AprilTagDetection detection = detections.get(0);
                telemetry.addLine("FOUND TAG");
                if(isTurnToTag) {
                    drive.turnWithPower((detection.center.x - 320.0) * (.003125 / 2));
                }
            }
            else {
                telemetry.addLine("TRYING TO DETECT");
            }


            telemetry.update();
            jp.update();
        }
    }
}
