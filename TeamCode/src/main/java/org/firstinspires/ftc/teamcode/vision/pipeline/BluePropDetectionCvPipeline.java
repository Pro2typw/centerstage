package org.firstinspires.ftc.teamcode.vision.pipeline;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.vision.util.TeamPropLocation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class BluePropDetectionCvPipeline extends OpenCvPipeline {
    public static int CENTER_RECTANGLE_TOP_LEFT_X = 0;
    public static int CENTER_RECTANGLE_TOP_LEFT_Y = 0;
    public static int CENTER_RECTANGLE_BOTTOM_RIGHT_X = 0;
    public static int CENTER_RECTANGLE_BOTTOM_RIGHT_Y = 0;

    public static int LEFT_RECTANGLE_TOP_LEFT_X = 0;
    public static int LEFT_RECTANGLE_TOP_LEFT_Y = 0;
    public static int LEFT_RECTANGLE_BOTTOM_RIGHT_X = 0;
    public static int LEFT_RECTANGLE_BOTTOM_RIGHT_Y = 0;

    public static int RIGHT_RECTANGLE_TOP_LEFT_X = 0;
    public static int RIGHT_RECTANGLE_TOP_LEFT_Y = 0;
    public static int RIGHT_RECTANGLE_BOTTOM_RIGHT_X = 0;
    public static int RIGHT_RECTANGLE_BOTTOM_RIGHT_Y = 0;

    private Mat testMat = new Mat();
    private Mat finalMat = new Mat();
    private final Scalar rectColor = new Scalar(255, 0, 0);

    private TeamPropLocation output = TeamPropLocation.LEFT;

    private static final Rect CENTER_RECTANGLE = new Rect(
            new Point(CENTER_RECTANGLE_TOP_LEFT_X, CENTER_RECTANGLE_TOP_LEFT_Y),
            new Point(CENTER_RECTANGLE_BOTTOM_RIGHT_X, CENTER_RECTANGLE_BOTTOM_RIGHT_Y)
    );

    private static final Rect LEFT_RECTANGLE = new Rect(
            new Point(LEFT_RECTANGLE_TOP_LEFT_X, LEFT_RECTANGLE_TOP_LEFT_Y),
            new Point(LEFT_RECTANGLE_BOTTOM_RIGHT_X, LEFT_RECTANGLE_BOTTOM_RIGHT_Y)
    );

    private static final Rect RIGHT_RECTANGLE = new Rect(
            new Point(RIGHT_RECTANGLE_TOP_LEFT_X, RIGHT_RECTANGLE_TOP_LEFT_Y),
            new Point(RIGHT_RECTANGLE_BOTTOM_RIGHT_X, RIGHT_RECTANGLE_BOTTOM_RIGHT_Y)
    );


    private double tolerance = 60;
    private final Scalar selectedValue = new Scalar(106, 144, 186);
    private double[] values = selectedValue.val;
    private Scalar lower = new Scalar(values[0] - tolerance < 0? 0: values[0] - tolerance,values[1] - tolerance < 0? 0: values[1] - tolerance, values[2] - tolerance < 0? 0: values[2] - tolerance);
    private Scalar upper = new Scalar(values[0] + tolerance > 255? 255: values[0] + tolerance,values[1] + tolerance > 255? 255: values[1] + tolerance, values[2] + tolerance > 255? 255: values[2] + tolerance);
    @Override
    public Mat processFrame(Mat frame) {
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);

        Core.inRange(testMat, lower, upper, finalMat);
        testMat.release();

        double centerBox = Core.sumElems(finalMat.submat(CENTER_RECTANGLE)).val[0];
        double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
        double rightBox = Core.sumElems(finalMat.submat(RIGHT_RECTANGLE)).val[0];

        double averagedCenterBox = centerBox / CENTER_RECTANGLE.area() / 255;
        double averagedLeftBox = leftBox / LEFT_RECTANGLE.area() / 255;
        double averagedRightBox = rightBox / RIGHT_RECTANGLE.area() / 255;

        double max = Math.max(Math.max(averagedLeftBox, averagedCenterBox), averagedRightBox);
        if(max == averagedLeftBox) output = TeamPropLocation.LEFT;
        else if(max == averagedCenterBox) output = TeamPropLocation.CENTER;
        else output = TeamPropLocation.RIGHT;

        Imgproc.rectangle(frame, CENTER_RECTANGLE, rectColor);
        Imgproc.rectangle(frame, LEFT_RECTANGLE, rectColor);
        Imgproc.rectangle(frame, RIGHT_RECTANGLE, rectColor);

        return frame;
    }
    public TeamPropLocation getPropPosition(){
        return output;
    }
}
