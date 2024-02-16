package org.firstinspires.ftc.teamcode.subsystem.vision.pipeline;
import org.firstinspires.ftc.teamcode.util.TeamPropLocation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpencvBluePropDetect extends OpenCvPipeline {

    private Mat testMat = new Mat();
    private Mat finalMat = new Mat();
    private final Scalar rectColor = new Scalar(255, 0, 0);


    private TeamPropLocation output = TeamPropLocation.LEFT;

    private static final Rect CENTER_RECTANGLE = new Rect(
            new Point(960 - 170, 360),
            new Point(960 - 565, 600)
    );

    private static final Rect LEFT_RECTANGLE = new Rect(
            new Point((960 - 730) / 2, 330),
            new Point(300, 600)
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

        double averagedCenterBox = centerBox / CENTER_RECTANGLE.area() / 255;
        double averagedRightBox = leftBox / LEFT_RECTANGLE.area() / 255;

        double max = Math.max(averagedRightBox, averagedCenterBox);
        if(max < .1) output = TeamPropLocation.LEFT;
        else if(max == averagedCenterBox) output = TeamPropLocation.CENTER;
        else output = TeamPropLocation.RIGHT;

        Imgproc.rectangle(frame, CENTER_RECTANGLE, rectColor);
        Imgproc.rectangle(frame, LEFT_RECTANGLE, rectColor);


        return frame;
    }
    public TeamPropLocation getPropPosition(){
        return output;
    }
}