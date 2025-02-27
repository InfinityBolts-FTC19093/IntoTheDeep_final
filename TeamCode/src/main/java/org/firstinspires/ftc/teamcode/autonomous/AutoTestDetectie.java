package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MECANUM.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.Collect;
import org.firstinspires.ftc.teamcode.actions.Prepare;
import org.firstinspires.ftc.teamcode.actions.Score;
import org.firstinspires.ftc.teamcode.autonomous.actions.CollectAuto;
import org.firstinspires.ftc.teamcode.autonomous.actions.PrepareAuto;
import org.firstinspires.ftc.teamcode.autonomous.actions.ScoreAuto;
import org.firstinspires.ftc.teamcode.autonomous.actions.actions;
import org.firstinspires.ftc.teamcode.constants.RobotMap;
import org.firstinspires.ftc.teamcode.systems.clawRotate_controller;
import org.firstinspires.ftc.teamcode.systems.claw_controller;
import org.firstinspires.ftc.teamcode.systems.linkage_controller;
import org.firstinspires.ftc.teamcode.systems.sliderClaw_controller;
import org.firstinspires.ftc.teamcode.systems.slider_controller;


@Autonomous (name = "TEST")
public class AutoTestDetectie extends LinearOpMode {
    slider_controller sliderController;
    claw_controller clawController;
    sliderClaw_controller sliderClawController;
    clawRotate_controller clawRotateController;
    linkage_controller linkageController;

    PrepareAuto SliderAction;
    CollectAuto LinkageAction;
    ScoreAuto Score;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotMap robot = new RobotMap(hardwareMap);
        Pose2d startPose = new Pose2d(9, -61.5, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        sliderController         = new slider_controller(robot.slider, hardwareMap);
        clawController           = new claw_controller(robot.claw);
        sliderClawController     = new sliderClaw_controller(robot.slider_claw);
        clawRotateController     = new clawRotate_controller(robot.claw_rotate);
        linkageController        = new linkage_controller(robot.linkage);

        SliderAction  = new PrepareAuto(robot.slider_claw, robot.slider_claw_tilt, robot.turret, robot.slider, robot.claw);
        LinkageAction = new CollectAuto(robot.claw ,robot.claw_tilt, robot.linkage, robot.claw_rotate, robot.claw_pivot);
        Score         = new ScoreAuto(robot.claw, robot.claw_tilt, robot.linkage, robot.claw_rotate, robot.claw_pivot, robot.slider_claw, robot.slider_claw_tilt, robot.turret, robot.slider, LinkageAction, SliderAction);

        actions.Update update = new actions.Update(hardwareMap);
        actions.scoreAuto score = new actions.scoreAuto(SliderAction, LinkageAction, Score);

        actions.scoreAuto.setSliderController(sliderController);

        if (!isStarted()) {
            update.initAll();
        }

        Action autoSequence = new SequentialAction(
                score.Basket()
        );

        Action pid = new ParallelAction(
          update.updateAll(), score.updateLift()
        );

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            Actions.runBlocking(new ParallelAction(
                            autoSequence, pid
                    )
            );
        }
    }
}
