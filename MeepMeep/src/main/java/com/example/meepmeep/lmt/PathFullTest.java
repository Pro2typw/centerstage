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


        final Pose2d StartingPose = new Pose2d(12, 63, Math.toRadians(90));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14.2, LENGTH )
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(0), 12.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartingPose)

                                .setReversed(true)
                                .addDisplacementMarker(() -> {
                                    // point toward backdrop ready to drop
//                                    robot.setArmState(Robot.ArmState.INTAKE);
                                })
                                .splineTo(new Vector2d(40, 36), Math.toRadians(0))
                                .addDisplacementMarker(() -> {
                                    // drop purple pixel
//                                    robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.OPEN);
                                })
                                .waitSeconds(1)
                                .lineTo(new Vector2d(41, 36))

                                .addDisplacementMarker(() -> {
                                    // pivot to backdrop with wrist
//                                    robot.claw.setClawState(Claw.ClawSide.LEFT, Claw.ClawState.CLOSE);
//                                    robot.setArmState(Robot.ArmState.TRANSITION);

                                })
                                .lineTo(new Vector2d(42, 36))
                                .waitSeconds(6)
                                .addDisplacementMarker(() -> {
//                                    robot.setArmState(Robot.ArmState.DEPO);
                                })
                                .lineTo(new Vector2d(55, 52))
                                .waitSeconds(2)
                                .addDisplacementMarker(() -> {
                                    // drop yellow pixel on backdrop
//                                    robot.claw.setClawState(Claw.ClawSide.RIGHT, Claw.ClawState.OPEN);
                                })
                                .waitSeconds(1)
                                .lineTo(new Vector2d(54.5, 52))
                                .addDisplacementMarker(() -> {
//                                    robot.setArmState(Robot.ArmState.TRANSITION);
                                })
                                .waitSeconds(2)
                                .lineTo(new Vector2d(54.4, 12))
                                .addDisplacementMarker(() -> {
//                                    robot.setArmState(Robot.ArmState.INTAKE);
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
