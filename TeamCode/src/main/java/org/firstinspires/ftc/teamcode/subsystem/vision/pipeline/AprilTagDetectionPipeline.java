package org.firstinspires.ftc.teamcode.subsystem.vision.pipeline;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystem.util.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagDetectionPipeline {
    private AprilTagProcessor aprilTagProcessor;
    
    public AprilTagDetectionPipeline() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
                .build();
    }
    
    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTagProcessor;
    }
    
    public List<AprilTagDetection> getAprilTagDetections() {
        return aprilTagProcessor.getDetections();
    }
    public Pose2d localize(double heading) {
//        Pose2d[] tagPoses = getIDPose(detection.id);
        List<AprilTagDetection> detections = getAprilTagDetections();
        
        if(detections.size() == 0) return null;
        
        double xSum = 0;
        double ySum = 0;
        double headingSum = 0;
        
        for(int i = 0; i < detections.size(); i++) {
            AprilTagDetection detection = detections.get(i);
            VectorF tagPosition = detection.metadata.fieldPosition;
            double xRelative = detection.ftcPose.x;
            double yRelative = detection.ftcPose.y;
            
            double xAbsolute = xRelative * Math.sin(Math.toRadians(heading)) + yRelative * Math.cos(Math.toRadians(heading));
            double yAbsolute = yRelative * Math.sin(Math.toRadians(heading)) - xRelative * Math.cos(Math.toRadians(heading));
            
            Pose2d pose = new Pose2d(
                    tagPosition.get(0) - xAbsolute - Constants.Camera.X_OFFSET,
                    tagPosition.get(1) - yAbsolute - Constants.Camera.Y_OFFSET,
                    heading
            );
            
            
        }
        return null; //  toodo change
    }
}
