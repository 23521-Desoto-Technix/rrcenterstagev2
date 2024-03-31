package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 30, Math.PI/2, Math.PI/2, 15)
                .build();

        /*myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                .splineTo(new Vector2d(12.5, 0), 0)
                .splineTo(new Vector2d(27, 5.5), Math.toRadians(45))
                .setReversed(true)
                .splineTo(new Vector2d(34.5, -33.5), Math.toRadians(-90))
                .build());*/
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(34.5, -33.5, Math.toRadians(-90)))
                .splineTo(new Vector2d(51, -35), Math.toRadians(90))
                .setReversed(true)
                .splineTo(new Vector2d(51, -40), Math.toRadians(90))
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}