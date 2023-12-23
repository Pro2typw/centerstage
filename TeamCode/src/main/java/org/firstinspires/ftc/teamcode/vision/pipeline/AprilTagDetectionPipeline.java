package org.firstinspires.ftc.teamcode.vision.pipeline;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.vision.util.CenterstageAprilTagPose;
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
    
    /**
     *
     * @param id the wanted april tag's id
     */
    public AprilTagDetection getAprilTagDetection(int id) {
        for(AprilTagDetection detection : aprilTagProcessor.getDetections()) {
            if(detection.id == id) return detection;
        }
        return null;
    }
    
    public List<AprilTagDetection> getAprilTagDetections() {
        return aprilTagProcessor.getDetections();
    }
    
    public Pose2d localize(AprilTagDetection detection) {
        return getIDPose(detection.id).minus(new Pose2d(detection.ftcPose.x, detection.ftcPose.y, Math.toRadians(detection.ftcPose.bearing)));
    }
    
    private Pose2d getIDPose(int id) {
        switch(id) {
            case 1:
                return CenterstageAprilTagPose.ID1;
            case 2:
                return CenterstageAprilTagPose.ID2;
            case 3:
                return CenterstageAprilTagPose.ID3;
            case 4:
                return CenterstageAprilTagPose.ID4;
            case 5:
                return CenterstageAprilTagPose.ID5;
            case 6:
                return CenterstageAprilTagPose.ID6;
                
            default: throw new IllegalArgumentException("Given April Tag ID is invalid");
            
        }
    }
}
