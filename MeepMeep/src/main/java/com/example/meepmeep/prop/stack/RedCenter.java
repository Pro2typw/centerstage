package com.example.meepmeep.prop.stack;

import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

public class RedCenter {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(62), 12)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setDimensions(15.8, 17.5)
                .build();

        Pose2d startPose = new Pose2d(-39, -61.5, Math.toRadians(90));

        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                // wrist init -> intake
                .lineToYSplineHeading(-36,  Math.toRadians(90))
                // drop purple
                .lineToYSplineHeading(-35, Math.toRadians(180))
                .setTangent(270)
                .lineToXSplineHeading(-54, Math.toRadians(180))
                //pickup white pixel from stack
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(-39, -58), Math.toRadians(0))


                .lineToX(22)

                .splineTo(new Vector2d(47,-34), Math.toRadians(0))


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
