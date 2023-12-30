package org.firstinspires.ftc.teamcode.subsystem.vision.util;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(group = "test")
public class VisionProcessorOpMode extends LinearOpMode {

    private VisionPortal portal;
    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        AprilTagProcessor pipeline = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();

        final CameraStreamProcessor processor = new CameraStreamProcessor(pipeline);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(processor)
                .enableLiveView(true)
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
