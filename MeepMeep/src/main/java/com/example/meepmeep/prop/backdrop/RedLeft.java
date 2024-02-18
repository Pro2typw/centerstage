package com.example.meepmeep.prop.backdrop;

import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

public class RedLeft {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setDimensions(15.8, 17.5)
                .build();

        Pose2d startPose = new Pose2d(15, -61.5, Math.toRadians(-90));

        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                // wrist init -> intake
                .lineToYSplineHeading(-29,  Math.toRadians(180))
                // drop purple
                .setTangent(Math.toRadians(180))
                .lineToX(45, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                        if(true) {
                            return 20;
                        }// arm isnt at 580
                        else {
                            return 40;
                        }
                    }})
                // if parking...
                .setTangent(Math.toRadians(90))
                .lineToY(-60)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
//                .setAxesInterval(10)
                .start();
    }
}
