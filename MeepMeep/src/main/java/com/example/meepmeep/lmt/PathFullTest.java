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


        final Pose2d startingPose = new Pose2d(-39, 63, Math.toRadians(90));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14.2, LENGTH )
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(0), 12.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startingPose)
                                .setReversed(true)
                                .lineTo(new Vector2d(-36, 51))
                                .lineToLinearHeading(new Pose2d(-36, 35, Math.toRadians(270)))
                                .addDisplacementMarker(() -> {
                                    // claw drop purple
                                })
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(-36, 60, Math.toRadians(180)))

                                .setReversed(true)
                                .lineTo(new Vector2d(12, 60))
                                .splineTo(new Vector2d(48, 36), 0)
                                .addDisplacementMarker(() -> {
                                    // pivot and drop yellow pixel on left backboard
                                    // park (just stay in front of backboard)
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
