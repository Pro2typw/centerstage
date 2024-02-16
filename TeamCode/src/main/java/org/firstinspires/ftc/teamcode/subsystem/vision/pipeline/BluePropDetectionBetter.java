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
    private Mat testMat = new Mat();
    private Mat finalMat = new Mat();
    private final Scalar rectColor = new Scalar(255, 0, 0);


    private TeamPropLocation output = TeamPropLocation.LEFT;

    private static final Rect CENTER_RECTANGLE = new Rect(
            new Point(640 - 170, 360),
            new Point(640 - 565, 480)
    );

    private static final Rect RIGHT_RECTANGLE = new Rect(
            new Point(640*.6, 200),
            new Point(640, 300)
    );

    private double tolerance = 60;
    private final Scalar selectedValue = new Scalar(106, 144, 186);
    private double[] values = selectedValue.val;
    private Scalar lower = new Scalar(values[0] - tolerance < 0? 0: values[0] - tolerance,values[1] - tolerance < 0? 0: values[1] - tolerance, values[2] - tolerance < 0? 0: values[2] - tolerance);
    private Scalar upper = new Scalar(values[0] + tolerance > 255? 255: values[0] + tolerance,values[1] + tolerance > 255? 255: values[1] + tolerance, values[2] + tolerance > 255? 255: values[2] + tolerance);
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }
    double averagedLeftBox, averagedRightBox;

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);

        Core.inRange(testMat, lower, upper, finalMat);
        testMat.release();

        double centerBox = Core.sumElems(finalMat.submat(CENTER_RECTANGLE)).val[0];
        double leftBox = Core.sumElems(finalMat.submat(RIGHT_RECTANGLE)).val[0];

        double averagedCenterBox = centerBox / CENTER_RECTANGLE.area() / 255;
        double averagedRightBox = leftBox / RIGHT_RECTANGLE.area() / 255;

        double max = Math.max(averagedRightBox, averagedCenterBox);
        if(max < .1) output = TeamPropLocation.LEFT;
        else if(max == averagedCenterBox) output = TeamPropLocation.CENTER;
        else output = TeamPropLocation.RIGHT;

        Imgproc.rectangle(frame, CENTER_RECTANGLE, rectColor);
        Imgproc.rectangle(frame, RIGHT_RECTANGLE, rectColor);


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
        return output;
    }

    public double[] getAveragedBoxes() {
        return new double[] {averagedLeftBox, averagedRightBox};
    }
}
