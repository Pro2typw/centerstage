package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

public class Camera {
    private VisionPortal portal;
    public Camera(HardwareMap hardwareMap, VisionProcessor ...processors) {
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessors(processors)
                .build();
    }


    public void setProcessorEnabled(VisionProcessor processor, boolean enabled) {
        portal.setProcessorEnabled(processor, enabled);
    }

    public boolean getProcessorEnabled(VisionProcessor processor) {
        return portal.getProcessorEnabled(processor);
    }

    public VisionPortal.CameraState getCameraState() {
        return portal.getCameraState();
    }

    public void saveNextFrameRaw(String filename) {
        portal.saveNextFrameRaw(filename);
    }

    public void stopStreaming() {
        portal.stopStreaming();
    }

    public void resumeStreaming() {
        portal.resumeStreaming();
    }

    public void stopLiveView() {
        portal.stopLiveView();
    }

    public void resumeLiveView() {
        portal.stopLiveView();
    }

    public float getFps() {
        return portal.getFps();
    }

    public <T extends CameraControl> T getCameraControl(Class<T> controlType) {
        return portal.getCameraControl(controlType);
    }

    public void setActiveCamera(WebcamName webcamName) {
        portal.setActiveCamera(webcamName);
    }

    public WebcamName getActiveCamera() {
        return portal.getActiveCamera();
    }

    public void close() {
        portal.close();
    }
}
