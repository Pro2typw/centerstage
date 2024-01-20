package com.example.meepmeep;

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

public class RightBluelm3version2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        final Pose2d StartingPose = new Pose2d(-36+5, 72-10, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14.2, 15.8)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(0), 12.5)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(StartingPose)
                                        .addDisplacementMarker(() -> {
//                                    arm.setState(Arm.ArmState.INTAKE);
                                        })
                                        .lineToLinearHeading(new Pose2d(-34.5, 31, Math.toRadians(180)))
                                        .addDisplacementMarker(() -> {
                                            // Place pixel on ground
//                                    claw.setLeftClawState(Claw.ClawState.OPEN);
                                        })
                                        .waitSeconds(2)
                                        .lineToLinearHeading(new Pose2d(-34.5, 12.5, Math.toRadians(180)))
                                        .addDisplacementMarker(() -> {
                                            // Place pixel on ground
//                                    claw.setLeftClawState(Claw.ClawState.OPEN);
                                        })
                                        .waitSeconds(1.5)
                                        .lineTo(new Vector2d(-55.5, 11.7))
                                        .addDisplacementMarker(() -> {
                                            // Place pixel on ground
//                                    claw.setLeftClawState(Claw.ClawState.OPEN);
                                        })
                                        .lineTo(new Vector2d(28, 11.3))
                                        .lineTo(new Vector2d(46, 27.4))
                                        .addDisplacementMarker(() -> {
                                            // Place pixel on ground
//                                    claw.setLeftClawState(Claw.ClawState.OPEN);
                                        })
                                        .strafeRight(13)
                                        .addDisplacementMarker(() -> {
                                            // Place pixel on ground
//                                    claw.setLeftClawState(Claw.ClawState.OPEN);
                                        })
                                        .waitSeconds(1.5)
                                        .lineTo(new Vector2d(28, 11.3))
                                        .lineTo(new Vector2d(-55, 11.3))
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
                                        .lineTo(new Vector2d(-55, 11.3))
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
