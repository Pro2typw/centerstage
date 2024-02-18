package org.firstinspires.ftc.teamcode.vision.util;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class PropDetectionUtil implements VisionProcessor {
    
    private int tolerance = 20;
    private Scalar selectedValue = new Scalar(87, 206, 96);; // hsv rgb(188, 42, 27) // ycrbg rgb(87, 206, 96)
    public Rect LEFT_ROI = new Rect(
            new Point(0, 0),
            new Point(200, 180)
    );
    public Rect CENTER_ROI = new Rect(
            new Point(20, 20),
            new Point(50, 50)
    );
    private double[] values = selectedValue.val;
    private Scalar lower = new Scalar(values[0] - tolerance < 0? 0: values[0] - tolerance,values[1] - tolerance < 0? 0: values[1] - tolerance, values[2] - tolerance < 0? 0: values[2] - tolerance);
    private Scalar upper = new Scalar(values[0] + tolerance > 255? 255: values[0] + tolerance,values[1] + tolerance > 255? 255: values[1] + tolerance, values[2] + tolerance > 255? 255: values[2] + tolerance);
    
    private Mat cvtMat = new Mat();
    private Mat binaryMat = new Mat();
    private Mat resultMat = new Mat();
    
    Telemetry telemetry;
    double error;
    
    public PropDetectionUtil(Telemetry telemetry) {
        this.telemetry = telemetry;
        error = 0;
    }
    
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    
    }
    
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, cvtMat, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(cvtMat, lower, upper, binaryMat);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(binaryMat, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(binaryMat, contours, -1, new Scalar(255,0,0));
        double maxArea = 0;
        MatOfPoint biggestContour = null;
        for(MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if(area > maxArea) {
                biggestContour = contour;
                maxArea = area;
            }
        }
        
        if(biggestContour != null) {
            Rect rect = Imgproc.boundingRect(biggestContour);
            
            resultMat.release();
            
            Core.bitwise_and(frame, frame, resultMat, binaryMat);
            Imgproc.rectangle(frame, rect, new Scalar(255,255,0));
            Imgproc.rectangle(frame, LEFT_ROI, new Scalar(255, 0, 0));
            Imgproc.rectangle(frame, CENTER_ROI, new Scalar(255, 0, 0));
            
            
            error = (rect.x + (rect.width / 2)) - (frame.width() / 2);
            telemetry.addData("Error", "(" + rect.x + ", " + rect.y + ") -> (" + (rect.x + rect.width) + ", " + (rect.y + rect.height) + ")");
            
            
            
            telemetry.update();
            
        }
        
        return null;
    }
    
    public double calculateIntersectionArea(Rect r1, Rect r2) {
        double leftX = Math.max(r1.x, r2.x);
        double topY  = Math.max(r1.y, r2.y);
        double rightX  = Math.min(r1.x + r1.width, r2.x + r2.width);
        double bottomY  = Math.min(r1.y + r1.height, r2.y + r2.height);
        
        double width = Math.max(0, rightX - leftX);
        double height = Math.max(0, bottomY - topY);
        
        return width * height;
    }
    
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    
    }
}
