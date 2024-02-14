package org.firstinspires.ftc.teamcode.subsystem.vision.pipeline;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.util.TeamPropLocation;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class BluePropDetectionBetter implements VisionProcessor {

    Mat testMat = new Mat();
    Mat highMat = new Mat();
    Mat lowMat = new Mat();
    Mat finalMat = new Mat();
    double redThreshold = 0.5;

    TeamPropLocation location = TeamPropLocation.LEFT;

    static final Rect LEFT_RECTANGLE = new Rect(
            new Point(0, 0),
            new Point(10, 10)
    );

    static final Rect RIGHT_RECTANGLE = new Rect(
            new Point(10, 10),
            new Point(20, 20)
    );

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }
    double averagedLeftBox, averagedRightBox;

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);


        Scalar lowHSVRedLower = new Scalar(0, 100, 20);
        Scalar lowHSVRedUpper = new Scalar(10, 255, 255);

        Scalar redHSVRedLower = new Scalar(160, 100, 20);
        Scalar highHSVRedUpper = new Scalar(180, 255, 255);

        Core.inRange(testMat, lowHSVRedLower, lowHSVRedUpper, lowMat);
        Core.inRange(testMat, redHSVRedLower, highHSVRedUpper, highMat);

        testMat.release();

        Core.bitwise_or(lowMat, highMat, finalMat);

        lowMat.release();
        highMat.release();

        double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
        double rightBox = Core.sumElems(finalMat.submat(RIGHT_RECTANGLE)).val[0];

        averagedLeftBox = leftBox / LEFT_RECTANGLE.area() / 255;
        averagedRightBox = rightBox / RIGHT_RECTANGLE.area() / 255;




        if(averagedLeftBox > redThreshold){
            location = TeamPropLocation.LEFT;
        }else if(averagedRightBox> redThreshold){
            location = TeamPropLocation.CENTER;
        }else{
            location = TeamPropLocation.RIGHT;
        }

        finalMat.copyTo(frame);
        return null;




    }


    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
//        Paint rectPaint = new Paint();
//        rectPaint.setColor(Color.BLUE);
//        rectPaint.setStyle(Paint.Style.STROKE);
//        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);
//
//        canvas.drawRect(makeGraphicsRect(LEFT_RECTANGLE, scaleBmpPxToCanvasPx), rectPaint);
//        canvas.drawRect(makeGraphicsRect(RIGHT_RECTANGLE, scaleBmpPxToCanvasPx), rectPaint);
////        canvas.drawRect(makeGraphicsRect(right, scaleBmpPxToCanvasPx), rectPaint);
    }

    public TeamPropLocation getPropPosition(){  //Returns postion of the prop in a String
        return location;
    }

    public double[] getAveragedBoxes() {
        return new double[] {averagedLeftBox, averagedRightBox};
    }
}
