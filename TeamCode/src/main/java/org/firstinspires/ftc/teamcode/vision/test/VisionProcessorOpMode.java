package org.firstinspires.ftc.teamcode.vision.test;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.pipeline.BluePropDetectionVisionProcessor;
import org.firstinspires.ftc.teamcode.vision.pipeline.RedPropDetectionVisionProcessor;
import org.firstinspires.ftc.teamcode.vision.util.CameraStreamProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

@TeleOp(group = "test")
public class VisionProcessorOpMode extends LinearOpMode {

    private VisionPortal portal;
    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // TODO: Add pipeline here
        // TODO: Check if this works
        VisionProcessor pipeline = new RedPropDetectionVisionProcessor();
        final CameraStreamProcessor processor = new CameraStreamProcessor(pipeline);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(processor)
                .build();

        FtcDashboard.getInstance().startCameraStream(processor, 0);

        while(opModeInInit()) {
            dashTelemetry.addData("Camera State", portal.getCameraState());
            dashTelemetry.addData("FPS", portal.getFps());
            dashTelemetry.update();
        }

        waitForStart();
    }

}
