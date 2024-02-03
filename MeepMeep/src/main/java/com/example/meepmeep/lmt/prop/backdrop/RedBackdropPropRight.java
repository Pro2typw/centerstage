package com.example.meepmeep.lmt.prop.backdrop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedBackdropPropRight {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        final double LENGTH = 18;


        final Pose2d StartingPose = new Pose2d(12, -63, Math.toRadians(270));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14.2, LENGTH - 4)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.5)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(StartingPose)
                                        .setReversed(true)
                                        .addDisplacementMarker(() -> {
                                            // point toward backdrop ready to drop
                                        })
                                        .splineTo(new Vector2d(32, 36 * -1), Math.toRadians(0))
                                        .lineTo(new Vector2d(10, 36 * -1))
                                        .addDisplacementMarker(() -> {
                                            // drop purple pixel
                                        })
                                        .waitSeconds(1)
                                        .addDisplacementMarker(() -> {
                                            // pivot to backdrop with wrist
                                        })
                                        .lineTo(new Vector2d(50, 36 * -1))
                                        .addDisplacementMarker(() -> {
                                            // drop yellow pixel on backdrop
                                            // rotate extension and pivot to match purple
                                            // drop purple pixel
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
