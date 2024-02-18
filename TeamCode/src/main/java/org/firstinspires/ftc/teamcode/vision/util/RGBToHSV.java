package org.firstinspires.ftc.teamcode.vision.util;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class RGBToHSV implements VisionProcessor {
    public int tolerance = 30;
    private final Scalar selectedValue = new Scalar(231, 130, 125);
    private double[] values = selectedValue.val;
    private Scalar lower = new Scalar(values[0] - tolerance < 0? 0: values[0] - tolerance,values[1] - tolerance < 0? 0: values[1] - tolerance, values[2] - tolerance < 0? 0: values[2] - tolerance);
    private Scalar upper = new Scalar(values[0] + tolerance > 255? 255: values[0] + tolerance,values[1] + tolerance > 255? 255: values[1] + tolerance, values[2] + tolerance > 255? 255: values[2] + tolerance);
    
    private Mat cvtMat = new Mat();
    private Mat binaryMat = new Mat();
    private Mat resultMat = new Mat();
    
    Telemetry telemetry;
    double error;
    
    public RGBToHSV(Telemetry telemetry) {
        this.telemetry = telemetry;
        error = 0;
    }
    
    
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    
    }
    
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2YCrCb);
        
        return null;
    }
    
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    
    }
}
