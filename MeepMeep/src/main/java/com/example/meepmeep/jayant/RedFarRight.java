package com.example.meepmeep.jayant;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedFarRight {
        public static void main(String[] args) {
            MeepMeep meepMeep = new MeepMeep(700);
            final double LENGTH = 18;


            final Pose2d StartingPose = new Pose2d(new Vector2d(-36, -63), Math.toRadians(270));


            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                    .setDimensions(14.2, LENGTH - 4)
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(0), 12.5)
                    .followTrajectorySequence(drive ->
                                    drive.trajectorySequenceBuilder(StartingPose)
                                            .lineToLinearHeading(new Pose2d(-36, -32, Math.toRadians(0)))
                                            .addDisplacementMarker(() -> {
                                                // Place preloaded pixel on tape
                                            })
                                            .waitSeconds(1)
                                            .lineToSplineHeading(new Pose2d(-52, -35, Math.toRadians(180)))
                                            .addDisplacementMarker(() -> {
                                                // Grab pixel from stack
                                            })
                                            .waitSeconds(1)

                                            .lineToSplineHeading(new Pose2d(-32, -58, Math.toRadians(180)))
                                            .lineTo(new Vector2d(11,-58))
                                            .splineTo(new Vector2d(48, -41), Math.toRadians(0))
                                            .addDisplacementMarker(() -> {
                                                // Place pixel on backboard
                                            })
                                            .waitSeconds(1)
                                            .setReversed(false)
                                            .splineTo(new Vector2d(11, -58), Math.toRadians(180))
                                            .lineTo(new Vector2d(-32,-58))
                                            .lineToSplineHeading(new Pose2d(-52, -35, Math.toRadians(180)))

// Cycle 2:
                                            .addDisplacementMarker(() -> {
                                                // Grab pixel from stack
                                            })
                                            .waitSeconds(1)
                                            .lineToSplineHeading(new Pose2d(-32, -58, Math.toRadians(180)))
                                            .lineTo(new Vector2d(11,-58))
                                            .splineTo(new Vector2d(48, -41), Math.toRadians(0))
                                            .addDisplacementMarker(() -> {
                                                // Place pixel on backboard
                                            })
                                            .waitSeconds(1)
// Park:
                                            .lineTo(new Vector2d(45, -41))

                                            .build()
                    );

            meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }

}
