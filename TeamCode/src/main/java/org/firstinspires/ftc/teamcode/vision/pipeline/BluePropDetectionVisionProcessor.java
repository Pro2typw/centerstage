package org.firstinspires.ftc.teamcode.vision.pipeline;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.TeamPropLocation;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class BluePropDetectionVisionProcessor implements VisionProcessor {
    private Mat testMat = new Mat();
    private Mat finalMat = new Mat();
    private final Scalar rectColor = new Scalar(0, 0, 255);

    private TeamPropLocation output = TeamPropLocation.LEFT;

    private static final Rect CENTER_RECTANGLE = new Rect(
            new Point(Constants.Vision.BLUE_CENTER_RECTANGLE_TOP_LEFT_X, Constants.Vision.BLUE_CENTER_RECTANGLE_TOP_LEFT_Y),
            new Point(Constants.Vision.BLUE_CENTER_RECTANGLE_BOTTOM_RIGHT_X, Constants.Vision.BLUE_CENTER_RECTANGLE_BOTTOM_RIGHT_Y)
    );

    private static final Rect LEFT_RECTANGLE = new Rect(
            new Point(Constants.Vision.BLUE_LEFT_RECTANGLE_TOP_LEFT_X, Constants.Vision.BLUE_LEFT_RECTANGLE_TOP_LEFT_Y),
            new Point(Constants.Vision.BLUE_LEFT_RECTANGLE_BOTTOM_RIGHT_X, Constants.Vision.BLUE_LEFT_RECTANGLE_BOTTOM_RIGHT_Y)
    );

    private static final Rect RIGHT_RECTANGLE = new Rect(
            new Point(Constants.Vision.BLUE_RIGHT_RECTANGLE_TOP_LEFT_X, Constants.Vision.BLUE_RIGHT_RECTANGLE_TOP_LEFT_Y),
            new Point(Constants.Vision.BLUE_RIGHT_RECTANGLE_BOTTOM_RIGHT_X, Constants.Vision.BLUE_RIGHT_RECTANGLE_BOTTOM_RIGHT_Y)
    );

    private double tolerance = 60;
    private final Scalar selectedValue = new Scalar(106, 144, 186);
    private double[] values = selectedValue.val;
    private Scalar lower = new Scalar(values[0] - tolerance < 0? 0: values[0] - tolerance,values[1] - tolerance < 0? 0: values[1] - tolerance, values[2] - tolerance < 0? 0: values[2] - tolerance);
    private Scalar upper = new Scalar(values[0] + tolerance > 255? 255: values[0] + tolerance,values[1] + tolerance > 255? 255: values[1] + tolerance, values[2] + tolerance > 255? 255: values[2] + tolerance);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);

        Core.inRange(testMat, lower, upper, finalMat);
        testMat.release();

        double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
        double centerBox = Core.sumElems(finalMat.submat(CENTER_RECTANGLE)).val[0];
        double rightBox = Core.sumElems(finalMat.submat(RIGHT_RECTANGLE)).val[0];

        double averagedLeftBox = leftBox / LEFT_RECTANGLE.area() / 255;
        double averagedCenterBox = centerBox / RIGHT_RECTANGLE.area() / 255;
        double averagedRightBox = rightBox / RIGHT_RECTANGLE.area() / 255;

        double max = Math.max(Math.max(averagedLeftBox, averagedCenterBox), averagedRightBox);
        if(max == averagedLeftBox) output = TeamPropLocation.LEFT;
        else if(max == averagedCenterBox) output = TeamPropLocation.CENTER;
        else output = TeamPropLocation.RIGHT;

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
        Rect left = new Rect(Constants.Vision.BLUE_LEFT_RECTANGLE_TOP_LEFT_Y, Constants.Vision.BLUE_LEFT_RECTANGLE_TOP_LEFT_X, Constants.Vision.BLUE_LEFT_RECTANGLE_BOTTOM_RIGHT_Y, Constants.Vision.BLUE_LEFT_RECTANGLE_BOTTOM_RIGHT_X);
        Rect center = new Rect(Constants.Vision.BLUE_CENTER_RECTANGLE_TOP_LEFT_Y, Constants.Vision.BLUE_CENTER_RECTANGLE_TOP_LEFT_X, Constants.Vision.BLUE_CENTER_RECTANGLE_BOTTOM_RIGHT_Y, Constants.Vision.BLUE_CENTER_RECTANGLE_BOTTOM_RIGHT_X);
        Rect right = new Rect(Constants.Vision.BLUE_RIGHT_RECTANGLE_TOP_LEFT_Y, Constants.Vision.BLUE_RIGHT_RECTANGLE_TOP_LEFT_X, Constants.Vision.BLUE_RIGHT_RECTANGLE_BOTTOM_RIGHT_Y, Constants.Vision.BLUE_RIGHT_RECTANGLE_BOTTOM_RIGHT_X);

        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.BLUE);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);

        canvas.drawRect(makeGraphicsRect(left, scaleBmpPxToCanvasPx), rectPaint);
        canvas.drawRect(makeGraphicsRect(center, scaleBmpPxToCanvasPx), rectPaint);
        canvas.drawRect(makeGraphicsRect(right, scaleBmpPxToCanvasPx), rectPaint);
    }

    public TeamPropLocation getPropPosition(){
        return output;
    }
}