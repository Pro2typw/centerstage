package org.firstinspires.ftc.teamcode.subsystem.vision.pipeline;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.subsystem.util.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystem.util.Constants;
import org.firstinspires.ftc.teamcode.util.TeamPropLocation;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.jetbrains.annotations.NotNull;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class PropDetectionPipeline implements VisionProcessor {

    private Mat testMat = new Mat();
    private Mat finalMat = new Mat();

    private TeamPropLocation output = TeamPropLocation.LEFT;

    private final Rect CENTER_RECTANGLE;
    private final Rect LEFT_RECTANGLE;
    private final Rect RIGHT_RECTANGLE;

    private final double tolerance;
    private final Scalar selectedValue;
    private double[] values;
    private Scalar lower;
    private Scalar upper;

    double averagedLeftBox = 0;
    double averagedCenterBox = 0;
    double averagedRightBox = 0;



    public PropDetectionPipeline(@NotNull AllianceColor color) {
        if(color == AllianceColor.BLUE) {

            CENTER_RECTANGLE = new Rect(
                    new Point(Constants.Vision.BLUE_CENTER_RECTANGLE_TOP_LEFT_X, Constants.Vision.BLUE_CENTER_RECTANGLE_TOP_LEFT_Y),
                    new Point(Constants.Vision.BLUE_CENTER_RECTANGLE_BOTTOM_RIGHT_X, Constants.Vision.BLUE_CENTER_RECTANGLE_BOTTOM_RIGHT_Y)
            );

            LEFT_RECTANGLE = new Rect(
                    new Point(Constants.Vision.BLUE_LEFT_RECTANGLE_TOP_LEFT_X, Constants.Vision.BLUE_LEFT_RECTANGLE_TOP_LEFT_Y),
                    new Point(Constants.Vision.BLUE_LEFT_RECTANGLE_BOTTOM_RIGHT_X, Constants.Vision.BLUE_LEFT_RECTANGLE_BOTTOM_RIGHT_Y)
            );

            RIGHT_RECTANGLE = new Rect(
                    new Point(Constants.Vision.BLUE_RIGHT_RECTANGLE_TOP_LEFT_X, Constants.Vision.BLUE_RIGHT_RECTANGLE_TOP_LEFT_Y),
                    new Point(Constants.Vision.BLUE_RIGHT_RECTANGLE_BOTTOM_RIGHT_X, Constants.Vision.BLUE_RIGHT_RECTANGLE_BOTTOM_RIGHT_Y)
            );

            tolerance = 60;
            selectedValue = new Scalar(106, 144, 186);
        }
        else {
            CENTER_RECTANGLE = new Rect(
                    new Point(Constants.Vision.RED_CENTER_RECTANGLE_TOP_LEFT_X, Constants.Vision.RED_CENTER_RECTANGLE_TOP_LEFT_Y),
                    new Point(Constants.Vision.RED_CENTER_RECTANGLE_BOTTOM_RIGHT_X, Constants.Vision.RED_CENTER_RECTANGLE_BOTTOM_RIGHT_Y)
            );

            LEFT_RECTANGLE = new Rect(
                    new Point(Constants.Vision.RED_LEFT_RECTANGLE_TOP_LEFT_X, Constants.Vision.RED_LEFT_RECTANGLE_TOP_LEFT_Y),
                    new Point(Constants.Vision.RED_LEFT_RECTANGLE_BOTTOM_RIGHT_X, Constants.Vision.RED_LEFT_RECTANGLE_BOTTOM_RIGHT_Y)
            );

            RIGHT_RECTANGLE = new Rect(
                    new Point(Constants.Vision.RED_RIGHT_RECTANGLE_TOP_LEFT_X, Constants.Vision.RED_RIGHT_RECTANGLE_TOP_LEFT_Y),
                    new Point(Constants.Vision.RED_RIGHT_RECTANGLE_BOTTOM_RIGHT_X, Constants.Vision.RED_RIGHT_RECTANGLE_BOTTOM_RIGHT_Y)
            );

            tolerance = 60;
            selectedValue = new Scalar(5, 185, 128);
        }
        values = selectedValue.val;
        lower = new Scalar(
                values[0] - tolerance < 0? 0: values[0] - tolerance,
                values[1] - tolerance < 0? 0: values[1] - tolerance,
                values[2] - tolerance < 0? 0: values[2] - tolerance);
        upper = new Scalar(
                values[0] + tolerance > 255? 255: values[0] + tolerance,
                values[1] + tolerance > 255? 255: values[1] + tolerance,
                values[2] + tolerance > 255? 255: values[2] + tolerance);

    }

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

        averagedLeftBox = leftBox / LEFT_RECTANGLE.area() / 255.0;
        averagedCenterBox = centerBox / RIGHT_RECTANGLE.area() / 255.0;
        averagedRightBox = rightBox / RIGHT_RECTANGLE.area() / 255.0;

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
