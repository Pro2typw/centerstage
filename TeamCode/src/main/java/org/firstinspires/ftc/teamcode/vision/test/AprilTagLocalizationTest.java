package org.firstinspires.ftc.teamcode.vision.test;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.pipeline.AprilTagDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(group = "test")
public class AprilTagLocalizationTest extends LinearOpMode {
    AprilTagDetectionPipeline processor;
    VisionPortal portal;
    
    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        processor = new AprilTagDetectionPipeline();
        
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(processor.getAprilTagProcessor())
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
        
        waitForStart();
        while(opModeIsActive()) {
            List<AprilTagDetection> detections = processor.getAprilTagDetections();
            telemetry.addData("Detections", detections.size());
            if(detections.size() != 0) {
                for(AprilTagDetection detection : detections) {
                    Pose2d estPos = processor.localize(detection);
                    telemetry.addLine("ID " + detection.metadata.id);
                    telemetry.addLine(String.format("X(%6.2f) Y(%6.2f) θ(%6.2f)", estPos.getX(), estPos.getY(), estPos.getX()));
                }
                
            }
            telemetry.update();
        }
    }
}
