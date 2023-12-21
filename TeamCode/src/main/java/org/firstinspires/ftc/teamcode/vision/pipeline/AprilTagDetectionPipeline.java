package org.firstinspires.ftc.teamcode.vision.pipeline;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagDetectionPipeline {
    private AprilTagProcessor aprilTagProcessor;
    
    public AprilTagDetectionPipeline() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
                .build();
    }
    
    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTagProcessor;
    }
    
    /**
     *
     * @param id the april tag's id
     */
    public Object getAprilTagDetection(int id) {
        for(AprilTagDetection detection : aprilTagProcessor.getDetections()) {
            if(detection.id == id) return detection;
            detection.ftcPose
        }
        return null;
    }
    
    public Pose2d localize(int id) {
        return new Pose2d();
    }
}
