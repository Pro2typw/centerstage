package org.firstinspires.ftc.teamcode.opmode.autonomus;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Hang;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.util.AllianceColor;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(group = "game")
public class BlueBackdrop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry, Claw.ClawState.CLOSE, Hang.HangState.DOWN, false, AllianceColor.BLUE);

        final Pose2d StartingPose = new Pose2d(12+5, 72-11.2, Math.toRadians(270));

        TrajectorySequence right = robot.drive.trajectorySequenceBuilder(StartingPose)
                .addDisplacementMarker(() -> {
//                                    arm.setState(Arm.ArmState.INTAKE);
                })
                .lineToLinearHeading(new Pose2d(30, 28.4, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    // Place pixel on backdrop
//                                    claw.setRightClawState(Claw.ClawState.OPEN);

                    // Place pixel on ground
//                                    claw.setLeftClawState(Claw.ClawState.OPEN);
                })
                .waitSeconds(3)
                .splineToLinearHeading(new Pose2d(8.5, 11.3, Math.toRadians(180)), Math.toRadians(180))
                .lineTo(new Vector2d(-34, 11.3))
                .addDisplacementMarker(() -> {
                    // Pickup pixel from stack
                })
                .waitSeconds(1.5)
                .lineTo(new Vector2d(8.5, 11.3))
                .splineToLinearHeading(new Pose2d(30, 34.6, Math.toRadians(180)), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    // Place pixel on backdrop
                })
                .waitSeconds(1.5)

                .splineToLinearHeading(new Pose2d(8.5, 11.3, Math.toRadians(180)), Math.toRadians(180))
                .lineTo(new Vector2d(-34, 11.3))
                .addDisplacementMarker(() -> {
                    // Pickup pixel from stack
                })
                .waitSeconds(1.5)
                .lineTo(new Vector2d(8.5, 11.3))
                .splineToLinearHeading(new Pose2d(30, 34.6, Math.toRadians(180)), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    // Place pixel on backdrop
                })
                .waitSeconds(1.5)

                .splineToLinearHeading(new Pose2d(57, 12, Math.toRadians(180)), Math.toRadians(0)) //Park
                .build();

        TrajectorySequence center = robot.drive.trajectorySequenceBuilder(StartingPose)
                .addDisplacementMarker(() -> {
//                                    arm.setState(Arm.ArmState.INTAKE);
                })
                .addDisplacementMarker(() -> {
                    // Place pixel on ground
//                                    claw.setLeftClawState(Claw.ClawState.OPEN);
                })
                .waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(34, 34.6, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    // Place pixel on backdrop
//                                    claw.setRightClawState(Claw.ClawState.OPEN);
                })
                .waitSeconds(1.5)
                .splineToLinearHeading(new Pose2d(25, 11.3, Math.toRadians(180)), Math.toRadians(180))
                .lineTo(new Vector2d(-34, 11.3))
                .addDisplacementMarker(() -> {
                    // Pickup pixel from stack
                })
                .waitSeconds(1.5)
                .lineTo(new Vector2d(8.5, 11.3))
                .splineToLinearHeading(new Pose2d(34, 28.4, Math.toRadians(180)), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    // Place pixel on backdrop
                })
                .waitSeconds(1.5)

                .splineToLinearHeading(new Pose2d(25, 11.3, Math.toRadians(180)), Math.toRadians(180))
                .lineTo(new Vector2d(-34, 11.3))
                .addDisplacementMarker(() -> {
                    // Pickup pixel from stack
                })
                .waitSeconds(1.5)
                .lineTo(new Vector2d(8.5, 11.3))
                .splineToLinearHeading(new Pose2d(34, 28.4, Math.toRadians(180)), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    // Place pixel on backdrop
                })
                .waitSeconds(1.5)

                .splineToLinearHeading(new Pose2d(57, 12, Math.toRadians(180)), Math.toRadians(0)) //Park
                .build();

        TrajectorySequence left = robot.drive.trajectorySequenceBuilder(StartingPose)
                .addDisplacementMarker(() -> {
//                                    arm.setState(Arm.ArmState.INTAKE);
                })
                .lineToLinearHeading(new Pose2d(40, 41, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    // Place pixel on backdrop
//                                    claw.setRightClawState(Claw.ClawState.OPEN);
                })
                .waitSeconds(1.5)
                .strafeLeft(12.6)
                .addDisplacementMarker(() -> {
                    // Place pixel on ground
//                                    claw.setLeftClawState(Claw.ClawState.OPEN);
                })
                .waitSeconds(1.5)
                .splineToLinearHeading(new Pose2d(25, 11.3, Math.toRadians(180)), Math.toRadians(180))
                .lineTo(new Vector2d(-34, 11.3))
                .addDisplacementMarker(() -> {
                    // Pickup pixel from stack
                })
                .waitSeconds(1.5)
                .lineTo(new Vector2d(25, 11.3))
                .splineToLinearHeading(new Pose2d(40, 28.4, Math.toRadians(180)), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    // Place pixel on backdrop
                })
                .waitSeconds(1.5)

                .splineToLinearHeading(new Pose2d(25, 11.3, Math.toRadians(180)), Math.toRadians(180))
                .lineTo(new Vector2d(-34, 11.3))
                .addDisplacementMarker(() -> {
                    // Pickup pixel from stack
                })
                .waitSeconds(1.5)
                .lineTo(new Vector2d(25, 11.3))
                .splineToLinearHeading(new Pose2d(40, 28.4, Math.toRadians(180)), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    // Place pixel on backdrop
                })
                .waitSeconds(1.5)

                .splineToLinearHeading(new Pose2d(57, 12, Math.toRadians(180)), Math.toRadians(0))
                .build();

        while (robot.camera.getCameraState() != VisionPortal.CameraState.STREAMING) {
            robot.telemetry.addLine("NOT READY");
            robot.telemetry.update();
            sleep(20);
        }

        robot.camera.setProcessorEnabled(robot.propDetectionPipeline, true);
        TrajectorySequence chosenSequence = center;

        while (opModeInInit()) {
            robot.telemetry.addData("Prop Location", robot.propDetectionPipeline.getPropPosition());
            robot.telemetry.update();
            switch (robot.propDetectionPipeline.getPropPosition()) {
                case LEFT:
                    chosenSequence = left;
                    break;
                case RIGHT:
                    chosenSequence = right;
                    break;
                default:
                    chosenSequence = center;
            }
        }

        waitForStart();

        robot.drive.followTrajectorySequence(chosenSequence);

        while (opModeIsActive()) {
            robot.telemetry.addData("Prop Location", robot.propDetectionPipeline.getPropPosition());
            robot.telemetry.update();

            robot.update();
        }

    }
}
