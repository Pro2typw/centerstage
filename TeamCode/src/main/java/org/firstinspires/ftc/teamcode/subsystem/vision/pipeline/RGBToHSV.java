package org.firstinspires.ftc.teamcode.subsystem.vision.pipeline;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RGBToHSV extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
        return input;
    }
}
