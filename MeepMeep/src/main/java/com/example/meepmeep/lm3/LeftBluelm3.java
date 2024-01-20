package com.example.meepmeep.lm3;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;



import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class LeftBluelm3 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        final Pose2d StartingPose = new Pose2d(12, 72-10, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14.2, 15.8)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(0), 12.5)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(StartingPose)
                                        .addDisplacementMarker(() -> {
//                                    arm.setState(Arm.ArmState.INTAKE);
                                        })
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

                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
