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
                .setTangent(Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(32, -40), Math.toRadians(90), null, new ProfileAccelConstraint(-60, 60))
                .splineToLinearHeading(new Pose2d(32, -14, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(50, -14, Math.toRadians(90)), Math.toRadians(90))

                .strafeToLinearHeading(new Vector2d(50, -50), Math.toRadians(90), null, new ProfileAccelConstraint(-90, 90))

                .splineToLinearHeading(new Pose2d(50, -16, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(62, -16, Math.toRadians(90)), Math.toRadians(90))

                .strafeToLinearHeading(new Vector2d(60, -50), Math.toRadians(90), null, new ProfileAccelConstraint(-90, 90))

                .strafeToLinearHeading(new Vector2d(60, -16), Math.toRadians(90), null, new ProfileAccelConstraint(-90, 90))
                .strafeToLinearHeading(new Vector2d(68, -16), Math.toRadians(90), null, new ProfileAccelConstraint(-90, 90))


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

                // SCHIMBA DIN CONSTANT HEADING IN LINEAR


                .strafeToLinearHeading(new Vector2d(63, -60), Math.toRadians(90))

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

                .splineToLinearHeading(new Pose2d(40, -55, Math.toRadians(0)), Math.toRadians(0))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}