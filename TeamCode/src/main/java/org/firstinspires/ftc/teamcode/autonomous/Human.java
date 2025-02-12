package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MECANUM.MecanumDrive;

public class Human extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        actions.Lift lift = new actions.Lift(hardwareMap);
        actions.Score score = new actions.Score(hardwareMap);
        actions.Claw claw = new actions.Claw(hardwareMap);

        /*


        traiectoriile + startPose-ul




        */

        Action autoSequence = new SequentialAction(

        );

        Action pid = new ParallelAction(
                lift.Update()
        );



        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        autoSequence, pid
                )
        );
    }
}
