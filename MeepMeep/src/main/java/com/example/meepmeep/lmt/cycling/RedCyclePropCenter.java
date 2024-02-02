package com.example.meepmeep.lmt.cycling;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedCyclePropCenter {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        final double LENGTH = 18;


        final Pose2d StartingPose = new Pose2d(50, 36, Math.toRadians(180));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14.2, LENGTH - 4)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(0), 12.5)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(StartingPose)
                                        .addDisplacementMarker(() -> {
                                            // pivot to intake
                                        })
                                        .lineTo(new Vector2d(-60, 36))
                                        .addDisplacementMarker(() -> {
                                            // claw close
                                        })
                                        .waitSeconds(1)
                                        .lineTo(new Vector2d(12, 36))
                                        .addDisplacementMarker(() -> {
                                            // depo position
                                        })
                                        .lineTo(new Vector2d(50, 36))
                                        .addDisplacementMarker(() -> {
                                            // claw drop yellow
                                        })
                                        .waitSeconds(1)
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
