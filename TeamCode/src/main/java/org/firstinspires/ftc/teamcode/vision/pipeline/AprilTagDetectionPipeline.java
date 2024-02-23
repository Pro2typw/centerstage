//package org.firstinspires.ftc.teamcode.vision.pipeline;
//
////import com.acmerobotics.roadrunner.geometry.Pose2d;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
//import org.firstinspires.ftc.teamcode.subsystem.Constants;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.List;
//
//public class AprilTagDetectionPipeline {
//    private AprilTagProcessor aprilTagProcessor;
//
//    public AprilTagDetectionPipeline() {
//        aprilTagProcessor = new AprilTagProcessor.Builder()
//                .setDrawAxes(true)
//                .setDrawTagOutline(true)
//                .setDrawTagID(true)
//                .setDrawCubeProjection(true)
//                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//                .build();
//    }
//
//    public AprilTagProcessor getAprilTagProcessor() {
//        return aprilTagProcessor;
//    }
//
//    public Pose2d localize(double headingRad) {
//        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
//
//        if(detections.size() == 0) return null;
//
//        double xSum = 0;
//        double ySum = 0;
//
//        for(AprilTagDetection detection : detections) {
//            Pose2d tagPos = vectorFToPose2d(detection.metadata.fieldPosition);
//            double tagHeading = quaternionToHeading(detection.metadata.fieldOrientation);
//
//            double x = detection.ftcPose.x - Constants.Camera.X_OFFSET;
//            double y = detection.ftcPose.y - Constants.Camera.Y_OFFSET;
//
////            headingRad = -headingRad;
//
//            double x2 = x * Math.cos(headingRad) + y * Math.sin(headingRad);
//            double y2 = x * -Math.sin(headingRad) + y * Math.cos(headingRad);
//
//            double absX = tagPos.getX() - y2;
//            double absY = tagPos.getY() + x2;
//
//            xSum += absX;
//            ySum += absY;
//        }
//
//        return new Pose2d(xSum / detections.size(), ySum / detections.size(), headingRad);
//
//    }
//
//    public static Pose2d vectorFToPose2d(VectorF vector) {
//        return new Pose2d(vector.get(0), vector.get(1));
//    }
//
//    public static double quaternionToHeading(Quaternion Q) {
//        return Math.atan2(2.0 * (Q.z * Q.w + Q.x * Q.y) , - 1.0 + 2.0 * (Q.w * Q.w + Q.x * Q.x));
//    }
//
//}