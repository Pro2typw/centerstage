package org.firstinspires.ftc.teamcode.subsystem.vision.test;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.subsystem.IMU;
import org.firstinspires.ftc.teamcode.subsystem.vision.pipeline.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.gamepad.JustPressed;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.inspection.GamepadInspection;

import java.util.concurrent.TimeUnit;

@TeleOp(group = "test")
public class AprilTagLocalizationTest extends LinearOpMode {
    AprilTagDetectionPipeline processor;
    VisionPortal portal;

    @Override
    public void runOpMode() throws InterruptedException {

        MultipleTelemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        JustPressed gp = new JustPressed(gamepad1);
        IMU imu = new IMU(hardwareMap);

        processor = new AprilTagDetectionPipeline();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(processor.getAprilTagProcessor())
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        while(portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            tele.addLine(portal.getCameraState().toString());
            tele.update();
            sleep(20);
        }

//        tele.addLine("READY: " + portal.getCameraState().toString());
//        tele.update();

        ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(100, TimeUnit.MILLISECONDS);

        GainControl gain = portal.getCameraControl(GainControl.class);
        gain.setGain(200);

        imu.resetAngularOrientation();

        while(opModeInInit()) {
            tele.addData("Exposure", exposure.getExposure(TimeUnit.MILLISECONDS));
            if(gp.b()) exposure.setExposure(exposure.getExposure(TimeUnit.MILLISECONDS) + 1, TimeUnit.MILLISECONDS);
            if(gp.a()) exposure.setExposure(exposure.getExposure(TimeUnit.MILLISECONDS) - 1, TimeUnit.MILLISECONDS);

            tele.addData("Gain", gain.getGain());
            if(gp.y()) gain.setGain(gain.getGain() + 1);
            if(gp.x()) gain.setGain(gain.getGain() - 1);

            tele.addLine("====" + processor.getAprilTagDetections().size() + "DETECTIONS====");

            if(processor.getAprilTagDetections().size() != 0) {
                tele.addLine("\n");
                tele.addLine(String.format("Raw Position: X(%6.2f) Y(%6.2f)", processor.getAprilTagDetections().get(0).metadata.fieldPosition.get(0), processor.getAprilTagDetections().get(0).metadata.fieldPosition.get(1)));
                tele.addLine("\n");
            }


            Pose2d estPos = processor.localize(tele, Math.toRadians(imu.getCurrentAngularOrientation().firstAngle));
            if(estPos != null) {
                tele.addLine(String.format("X(%6.2f) Y(%6.2f) θ(%6.2f)", estPos.getX(), estPos.getY(), imu.getCurrentAngularOrientation().firstAngle));

            }
            tele.update();
            gp.update();
            sleep(20);
        }


        waitForStart();
    }
}
