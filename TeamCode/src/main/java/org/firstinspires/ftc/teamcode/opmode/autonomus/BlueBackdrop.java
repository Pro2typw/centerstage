package org.firstinspires.ftc.teamcode.opmode.autonomus;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Hang;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Wrist;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.TeamPropLocation;

@Autonomous(group = "game", name = "2+0 blue backdrop")
public class BlueBackdrop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry, Claw.ClawState.CLOSE, Hang.HangState.DOWN);
        TeamPropLocation loc = TeamPropLocation.LEFT;

        TrajectorySequence left = robot.drive.trajectorySequenceBuilder(new Pose2d())

                //insert
                .lineToLinearHeading(new Pose2d(46, 40, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    // Place pixel on backdrop
//                                    claw.setRightClawState(Claw.ClawState.OPEN);
                })
                .waitSeconds(1.5)
                .strafeLeft(12.6)
                .forward(8.5)
                .addDisplacementMarker(() -> {
                    // Place pixel on ground
//                                    claw.setLeftClawState(Claw.ClawState.OPEN);
                })
                .waitSeconds(1.5)
                .lineTo(new Vector2d(28, 11.3))
                .lineTo(new Vector2d(-50, 11.3))
                .addDisplacementMarker(() -> {
                    // Pickup pixel from stack
                })
                .waitSeconds(1.5)
                .lineTo(new Vector2d(25, 11.3))
                .lineTo(new Vector2d(46, 27.4))
                .addDisplacementMarker(() -> {
                    // Place pixel on backdrop
                })
                .waitSeconds(1.5)

                .lineTo(new Vector2d(25, 11.3))
                .lineTo(new Vector2d(-50, 11.3))
                .addDisplacementMarker(() -> {
                    // Pickup pixel from stack
                })
                .waitSeconds(1.5)
                .lineTo(new Vector2d(25, 11.3))
                .lineTo(new Vector2d(46, 27.4))
                .addDisplacementMarker(() -> {
                    // Place pixel on backdrop
                })
                .waitSeconds(1.5)

                .lineTo(new Vector2d(57, 12))

                .build();
        TrajectorySequence right = robot.drive.trajectorySequenceBuilder(new Pose2d())
                //insert
                .back(5)
                .addDisplacementMarker(() -> {
//                                    arm.setState(Arm.ArmState.INTAKE);
                    robot.wrist.setState(Wrist.WristState.INTAKE_POS);
                    robot.arm.setPivotTargetPos(0);
                })
                .lineToLinearHeading(new Pose2d(12, 72-10-16, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(15, 30, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    // Place pixel on backdrop
//                                    claw.setRightClawState(Claw.ClawState.OPEN);
                    robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.OPEN);
                    // Place pixel on ground
//                                    claw.setLeftClawState(Claw.ClawState.OPEN);
                })
                .waitSeconds(3)
                .forward(21)
                .addDisplacementMarker(() -> {
                    //place pixel on background
                })
                .waitSeconds(2)
                .lineTo(new Vector2d(8.5, 11.3))
                .lineTo(new Vector2d(-55, 11.3))
                .addDisplacementMarker(() -> {
                    // Pickup pixel from stack
                })
                .waitSeconds(1.5)
                .lineTo(new Vector2d(8.5, 11.3))
                .lineTo(new Vector2d(45, 40.5))
                .addDisplacementMarker(() -> {
                    // Place pixel on backdrop
                })
                .waitSeconds(1.5)

                .lineTo(new Vector2d(8.5, 11.3))
                .lineTo(new Vector2d(-55, 11.3))
                .addDisplacementMarker(() -> {
                    // Pickup pixel from stack
                })
                .waitSeconds(1.5)
                .lineTo(new Vector2d(8.5, 11.3))
                .lineTo(new Vector2d(45, 40.5))
                .addDisplacementMarker(() -> {
                    // Place pixel on backdrop
                })
                .waitSeconds(1.5)

                .lineTo(new Vector2d(57, 12)) //Park

                .build();
        TrajectorySequence center = robot.drive.trajectorySequenceBuilder(new Pose2d())
                .back(5)
                .lineToLinearHeading(new Pose2d(12, 72-10-6, Math.toRadians(270)))
                .addDisplacementMarker(() -> {
//                                    arm.setState(Arm.ArmState.INTAKE);
                    robot.wrist.setState(Wrist.WristState.INTAKE_POS);
                    robot.arm.setPivotTargetPos(0);
                })
                .forward(21) //(72-11.2 - 34.6)
                .addDisplacementMarker(() -> {
                    // Place pixel on ground
//                                    claw.setLeftClawState(Claw.ClawState.OPEN);
                    robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.OPEN);
                })
                .waitSeconds(1.5)
                .addDisplacementMarker(() -> {
                    robot.arm.setExtensionTargetPos(600);
                })
                .waitSeconds(.5)
                .addDisplacementMarker(() -> {
                    robot.wrist.setState(Wrist.WristState.LOW_DEPO_POS);
                })
                .lineToLinearHeading(new Pose2d(46, 34.6, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    // Place pixel on backdrop
//                                    claw.setRightClawState(Claw.ClawState.OPEN);
                    robot.claw.setClawState(Claw.ClawSide.RIGHT, Claw.ClawState.OPEN);
                })
//                .waitSeconds(1.5)
//                .lineTo(new Vector2d(25, 11.3))
//                .lineTo(new Vector2d(-56, 11.3))
//                .addDisplacementMarker(() -> {
//                    // Pickup pixel from stack
//                    robot.arm.setPivotTargetPos(45);
//                })
//                .waitSeconds(1.5)
//                .lineTo(new Vector2d(8.5, 11.3))
//                .lineTo(new Vector2d(44.5, 18))
//                .strafeRight(10)
//                .addDisplacementMarker(() -> {
//                    // Place pixel on backdrop
//                    robot.arm.setPivotTargetPos(600);
//                })
                .waitSeconds(1.5)

                .lineTo(new Vector2d(57, 12)) //Park

                .build();


        do {
            if(loc == TeamPropLocation.LEFT) loc = TeamPropLocation.CENTER;
            else if(loc == TeamPropLocation.CENTER) loc = TeamPropLocation.RIGHT;
            else loc = TeamPropLocation.LEFT;
            telemetry.addData("PROP Location", loc);
            telemetry.update();
            sleep(1000);
        } while (opModeInInit());

        waitForStart();
        robot.init();

        switch (loc) {
            case LEFT:
                robot.drive.followTrajectorySequence(left);
                break;
            case RIGHT:
                robot.drive.followTrajectorySequence(right);
                break;
            case CENTER:
                robot.drive.followTrajectorySequence(center);
                break;

        }

        while (opModeIsActive()) {
            robot.update();
        }
    }
}
