package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class basket {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 9.5)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-33, -61.5, Math.toRadians(0)))
                //preload
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))

                //take1+place
                .strafeToLinearHeading(new Vector2d(-48, -44), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))

                //take2+place
                .strafeToLinearHeading(new Vector2d(-58, -44), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))

                //take3+place
                .strafeToLinearHeading(new Vector2d(-55, -44), Math.toRadians(125))
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))

                //park
                .strafeToLinearHeading(new Vector2d(-35, -10), Math.toRadians(0))
                .strafeTo(new Vector2d(-20, -10))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}