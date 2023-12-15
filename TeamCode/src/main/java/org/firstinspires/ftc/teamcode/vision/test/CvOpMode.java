package org.firstinspires.ftc.teamcode.vision.test;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.pipeline.BluePropDetectionCvPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(group = "test")
public class CvOpMode extends LinearOpMode {

    WebcamName webcamName;

    @Override
    public void runOpMode() throws InterruptedException {

        // TODO: Add pipeline here
        // TODO: Check if this works
        OpenCvPipeline pipeline = new BluePropDetectionCvPipeline();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera cvCamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        cvCamera.setPipeline(pipeline);
        cvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cvCamera.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        FtcDashboard.getInstance().startCameraStream(cvCamera, 0);


        while (opModeInInit()) {
            telemetry.addData("Camera", webcamName.toString());
            telemetry.addData("Theoretical Max FPS", cvCamera.getCurrentPipelineMaxFps());
            telemetry.addData("Pipeline Time (ms)", cvCamera.getPipelineTimeMs());
            telemetry.addData("Overhead Time (ms)", cvCamera.getOverheadTimeMs());
            telemetry.addData("Total Frame Time (ms)", cvCamera.getTotalFrameTimeMs());
            telemetry.update();
        }

        waitForStart();
    }
}
