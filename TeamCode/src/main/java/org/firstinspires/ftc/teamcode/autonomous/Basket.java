package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MECANUM.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.Collect;
import org.firstinspires.ftc.teamcode.actions.Prepare;
import org.firstinspires.ftc.teamcode.actions.Score;
import org.firstinspires.ftc.teamcode.constants.RobotMap;

public class Basket extends LinearOpMode {
    Collect LinkageAction;
    Prepare SliderAction;
    Score ScoreAction;
    RobotMap robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotMap(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);


        SliderAction     = new Prepare(robot.slider_claw, robot.slider_claw_tilt, robot.turret, robot.slider, robot.claw);
        LinkageAction    = new Collect(robot.claw ,robot.claw_tilt, robot.linkage, robot.claw_rotate, robot.claw_pivot);
        ScoreAction      = new Score(robot.claw, robot.claw_tilt, robot.linkage, robot.claw_rotate, robot.claw_pivot, robot.slider_claw, robot.slider_claw_tilt, robot.turret, robot.slider, LinkageAction, SliderAction);

        actions.Update updateAuto   = new actions.Update(hardwareMap);
        actions.scoreAuto scoreAuto = new actions.scoreAuto(ScoreAction, SliderAction, LinkageAction);
        actions.Lift lift           = new actions.Lift(hardwareMap);

        /*


        traiectoriile + startPose-ul




        */

        Action autoSequence = new SequentialAction(
        );

        Action pid = new ParallelAction(
                updateAuto.updateAll()
        );



        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        autoSequence, pid
                )
        );
    }
}
