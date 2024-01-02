package org.firstinspires.ftc.teamcode.subsystem.vision.test;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystem.IMU;
import org.firstinspires.ftc.teamcode.subsystem.vision.pipeline.AprilTagDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(group = "test")
public class AprilTagLocalizationTest extends LinearOpMode {
    AprilTagDetectionPipeline processor;
    VisionPortal portal;
    
    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        IMU imu = new IMU(hardwareMap);
        imu.resetAngularOrientation();
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
            Pose2d estPos = processor.localize(imu.getCurrentAngularOrientation().firstAngle);
            if(estPos != null) {
                tele.addLine(String.format("X(%6.2f) Y(%6.2f) θ(%6.2f)", estPos.getX(), estPos.getY(), estPos.getX()));
                
            }
            tele.update();
        }
    }
}