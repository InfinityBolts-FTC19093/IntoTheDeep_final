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
                .strafeTo(new Vector2d(8, -40))

                //da elementele la human
                                .strafeToLinearHeading(new Vector2d(34, -34), Math.toRadians(45))
                                .turn(Math.toRadians(-100))
                                .turn(Math.toRadians(100))
                                .strafeToLinearHeading(new Vector2d(40, -34), Math.toRadians(45))
                                .turn(Math.toRadians(-100))
                                .turn(Math.toRadians(100))
                                .strafeToLinearHeading(new Vector2d(46, -34) ,Math.toRadians(45))
                                .turn(Math.toRadians(-100))


                /*
                .strafeToLinearHeading(new Vector2d(35, -35), Math.toRadians(90))
                .strafeTo(new Vector2d(35, -13))
                .strafeTo(new Vector2d(45, -13))
                .strafeTo(new Vector2d(45, -57))

                .strafeTo(new Vector2d(43, -13), null, new ProfileAccelConstraint(-100, 100))

                .strafeTo(new Vector2d(57, -13), null, new ProfileAccelConstraint(-70, 70))
                .strafeTo(new Vector2d(57, -57), null, new ProfileAccelConstraint(-100, 100))

                        .strafeTo(new Vector2d(50, -13))
                        .strafeTo(new Vector2d(65, -13))

*/
                //merge la human
                .strafeToLinearHeading(new Vector2d(50, -60), Math.toRadians(90))

                //pune primu
                .strafeToConstantHeading(new Vector2d(-4, -35), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(-4, -34))


                //human
                .strafeToConstantHeading(new Vector2d(45, -55), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(45, -61))


                //al doilea
                .strafeToConstantHeading(new Vector2d(0, -35), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(0, -32))


                //human
                .strafeToConstantHeading(new Vector2d(45, -55), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(45, -61))


                //al treilea
                .strafeToConstantHeading(new Vector2d(4, -35), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(4, -32))


                //human
                .strafeToConstantHeading(new Vector2d(45, -55), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(45, -61))


                //al patrulea
                .strafeToConstantHeading(new Vector2d(8, -35), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(8, -32))



                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}