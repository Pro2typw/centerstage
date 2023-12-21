package org.firstinspires.ftc.teamcode.vision.test;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.pipeline.AprilTagDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(group = "test")
public class AprilTagLocalization extends LinearOpMode {
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
            if(processor.getAprilTagDetections().size() != 0) {
                AprilTagDetection detection = processor.getAprilTagDetections().get(0);
                tele.addLine(String.format("XYZ %6.2f %6.2f %6.2f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                tele.addLine(String.format("XYZ %6.2f %6.2f %6.2f", detection.rawPose.x, detection.rawPose.y, detection.rawPose.z));
                
            }
            
        }
    }
}
