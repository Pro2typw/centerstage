package com.example.meepmeep.lmt;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PathFullTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        final double LENGTH = 18;


        final Pose2d StartingPose = new Pose2d(12, -63, Math.toRadians(270));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14.2, LENGTH )
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(0), 12.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartingPose)
                                .setReversed(true)
                                .addDisplacementMarker(() -> {
                                    // point toward backdrop ready to drop
//                                    robot.wrist.setPosition(Constants.Wrist.INTAKE_POS);
                                })
                                .lineTo(new Vector2d(15, -51))
                                .lineToLinearHeading(new Pose2d(15, -33, Math.toRadians(270)))
                                .addDisplacementMarker(() -> {
                                    // claw drop purple
//                                    robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.OPEN);
                                })
                                .waitSeconds(.5)
                                .lineTo(new Vector2d(12.1, -36))
                                .addDisplacementMarker(() -> {
//                                    robot.arm.setPivotTargetPos(150);
                                })
                                .waitSeconds(1)
                                .lineTo(new Vector2d(36, -36))
                                .addDisplacementMarker(() -> {
                                    // pivot to depo
//                                    robot.arm.setPivotTargetPos(580);
//                                    robot.wrist.setPosition(Constants.Wrist.DEPO_POS);
//                                    robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.CLOSE);
                                })
                                .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(180)))
                                .addDisplacementMarker(() -> {
                                    // claw drop yellow
//                                    robot.claw.setClawState(Claw.ClawSide.RIGHT, Claw.ClawState.OPEN);
                                })
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
