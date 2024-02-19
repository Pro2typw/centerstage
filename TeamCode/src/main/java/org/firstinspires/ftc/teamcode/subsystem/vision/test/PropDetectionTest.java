package org.firstinspires.ftc.teamcode.subsystem.vision.test;

import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.subsystem.util.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystem.Camera;
import org.firstinspires.ftc.teamcode.subsystem.vision.pipeline.BluePropDetectionBetter;
import org.firstinspires.ftc.teamcode.subsystem.vision.pipeline.PropDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.gamepad.JustPressed;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

@TeleOp(group = "test")
public class PropDetectionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BluePropDetectionBetter pipeline = new BluePropDetectionBetter();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
//                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(pipeline)

                .build();

        portal.setProcessorEnabled(pipeline, true);

        while (opModeInInit()) {
//            telemetry.addData("Prop Position", pipeline);
            double[] avg = pipeline.getAveragedBoxes();
            telemetry.addLine(avg[0] + " " + avg[1]);
            telemetry.update();
        }
        waitForStart();
    }
}
