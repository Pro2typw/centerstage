package org.firstinspires.ftc.teamcode.opmode.game.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.rr.commands.RobotC;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.util.AllianceStates;

@Autonomous(group = "0")
public class BlueStack0 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, telemetry, Claw.ClawState.CLOSE);
        RobotC robotC = new RobotC(robot);
        Pose2d startPose = new Pose2d(-39, 61.5, Math.toRadians(90));

        robot.drive = new MecanumDrive(hardwareMap, startPose);

        Action left1 = robot.drive.actionBuilder(robot.drive.pose)
                .lineToYSplineHeading(36,  Math.toRadians(0))
                .setTangent(Math.toRadians(180))
                .lineToX(-33)
                .build();

        Action left2 = robot.drive.actionBuilder(new Pose2d(-33, 36, 0))
                .lineToX(-45)
                .setTangent(90)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .setTangent(Math.toRadians(180))
                .lineToXSplineHeading(-56.25, Math.toRadians(180), new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 20;
                    }
                })
                .build();

        Action left3 = robot.drive.actionBuilder(new Pose2d(-56, 33, Math.toRadians(180)))
                .setTangent(0)
                .lineToX(-51, (pose2dDual, posePath, v) -> 10)
                .splineToConstantHeading(new Vector2d(-39, 61.5), Math.toRadians(90))
                .setTangent(Math.toRadians(180))
                .lineToX(22)
                .build();

        Action left4 = robot.drive.actionBuilder(new Pose2d(22, 61.5, Math.toRadians(180)))
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(47,34), Math.toRadians(0))
                .build();


        Action center1 = robot.drive.actionBuilder(startPose)
                .lineToYSplineHeading(33,  Math.toRadians(270))
                .setTangent(Math.toRadians(180))
                .lineToX(-35)
//                .setTangent(Math.toRadians(270))
                .build();

        Action center2 = robot.drive.actionBuilder(new Pose2d(-39, 33, Math.toRadians(270)))
                .lineToY(40)
                .build();



        telemetry.addLine("PATHS BUILT");
        telemetry.update();

        robot.wrist.setPosition(Constants.Wrist.INIT_POS);
        robot.arm.setPivotTargetPos(0);
        robot.arm.setExtensionTargetPos(0);

        AllianceStates location = AllianceStates.LEFT;
        while(opModeInInit() && !isStopRequested()) {
            switch (Math.round(System.currentTimeMillis() / 1000) % 3) {
                case 0:
                    location = AllianceStates.LEFT;
                    break;
                case 1:
                    location = AllianceStates.RIGHT;
                    break;
                case 2:
                    location = AllianceStates.CENTER;
                    break;
            }

            // todo remove
            location = AllianceStates.CENTER;

            telemetry.addData("Location", location);
            robot.update();
        }

        waitForStart();
        
        if(location == AllianceStates.LEFT) {
            Actions.runBlocking(
                    new ParallelAction(
                            new SequentialAction(
                                    new ParallelAction(
                                            left1, // turn to purple
                                            robotC.setWristPosition(Constants.Wrist.INTAKE_POS)
                                    ),
                                    robotC.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.OPEN),
//                                    robotC.waitSeconds(1000),
                                    new ParallelAction(
                                            left2, // move to badckdrop
                                            robotC.setPivotToStack(48)
                                    ),
                                    robotC.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.CLOSE),
                                    left3,
                                    left4,
                                    new ParallelAction(
                                            robotC.setPivotPosition(580),
                                            robotC.setWristPosition(Constants.Wrist.DEPO_POS_580)
                                    )
                            ),
                            robotC.update()
                    )
            );
        }
        else if(location == AllianceStates.CENTER) {
            Actions.runBlocking(
                    new ParallelAction(
                            new SequentialAction(
                                    new ParallelAction(
                                            center1,
                                            robotC.setWristPosition(Constants.Wrist.INTAKE_POS)
                                    ),
                                    robotC.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.OPEN),
                                    center2
                            ),
                            robotC.update()
                    )
            );
        }
        
    }
}
