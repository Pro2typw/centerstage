package com.example.meepmeep.lmt.prop.stack;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueBackdropPropCenter {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        final double LENGTH = 18;


        final Pose2d StartingPose = new Pose2d(-36, 72-LENGTH/2, Math.toRadians(90));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14.2, LENGTH - 4)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(0), 12.5)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(StartingPose)
                                        .setReversed(true)
                                        .lineTo(new Vector2d(-36, 51))
                                        .lineToLinearHeading(new Pose2d(-36, 33, Math.toRadians(270)))
                                        .addDisplacementMarker(() -> {
                                            // claw drop purple
                                        })
                                        .waitSeconds(1)
                                        .back(3)
                                        .addDisplacementMarker(() -> {
                                            // pivot to stack for one
                                        })
                                        .lineToLinearHeading(new Pose2d(-59, 36, Math.toRadians(180)))
                                        .addDisplacementMarker(() -> {
                                            // claw pickup from stack
                                        })
                                        .waitSeconds(1)
                                        .lineTo(new Vector2d(0, 36))
                                        .addDisplacementMarker(() -> {
                                            // pivot to depo
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
