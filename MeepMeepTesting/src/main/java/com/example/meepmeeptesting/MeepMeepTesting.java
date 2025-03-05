package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 9.5)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(9, -61.5, Math.toRadians(90)))
                .strafeTo(new Vector2d(8, -33), null, new ProfileAccelConstraint(-70, 70))
                                .strafeTo(new Vector2d(8, -35))

                //da elementele la human
                                .splineToLinearHeading(new Pose2d(30, -38, Math.toRadians(40)), Math.toRadians(90))
                                .strafeToLinearHeading(new Vector2d(30, -40), Math.toRadians(300))
                                .strafeToLinearHeading(new Vector2d(38, -38), Math.toRadians(40))
                .strafeToLinearHeading(new Vector2d(38, -40), Math.toRadians(300))
                .strafeToLinearHeading(new Vector2d(46, -38), Math.toRadians(30))
                .strafeToLinearHeading(new Vector2d(46, -40), Math.toRadians(300))





                /* SCHIMBA DIN CONSTANT HEADING IN LINEAR */


                .strafeToLinearHeading(new Vector2d(45, -60), Math.toRadians(90))

                //pune primu
                .strafeToConstantHeading(new Vector2d(6, -35), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(6, -34))


                //human
                .strafeToConstantHeading(new Vector2d(45, -55), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(45, -61))


                //al doilea
                .strafeToConstantHeading(new Vector2d(8, -35), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(8, -32))


                //human
                .strafeToConstantHeading(new Vector2d(45, -55), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(45, -61))


                //al treilea
                .strafeToConstantHeading(new Vector2d(10, -35), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(10, -32))


                //human
                .strafeToConstantHeading(new Vector2d(45, -55), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(45, -61))


                //al patrulea
                .strafeToConstantHeading(new Vector2d(12, -35), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(12, -32))

//                        .strafeTo(new Vector2d(12, -40))
//                        .strafeToLinearHeading(new Vector2d(40, -55), Math.toRadians(0))

                .strafeToLinearHeading(new Vector2d(35, -45),Math.toRadians(330))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}