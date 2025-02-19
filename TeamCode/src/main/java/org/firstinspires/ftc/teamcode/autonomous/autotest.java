package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MECANUM.MecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.actions.actionsManual;
import org.firstinspires.ftc.teamcode.constants.RobotMap;

@Autonomous (name = "auto")
public class autotest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        RobotMap robot = new RobotMap(hardwareMap);
        Pose2d startPose = new Pose2d(-33, -61.5, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        actionsManual.Lift lift = new actionsManual.Lift(hardwareMap);
        actionsManual.SliderClaw sliderClaw = new actionsManual.SliderClaw(hardwareMap);
        actionsManual.Linkage linkage = new actionsManual.Linkage(hardwareMap);
        actionsManual.Claw claw = new actionsManual.Claw(hardwareMap);
        actionsManual.ClawRotate clawRotate = new actionsManual.ClawRotate(hardwareMap);
        actionsManual.SliderTilt sliderTilt = new actionsManual.SliderTilt(hardwareMap);
        actionsManual.Turret turret = new actionsManual.Turret(hardwareMap);
        actionsManual.ClawTilt clawTilt = new actionsManual.ClawTilt(hardwareMap);
        actionsManual.Pivot pivot = new actionsManual.Pivot(hardwareMap);
        

        Action TakeFromGround = new SequentialAction(
                linkage.take(), claw.open(), clawTilt.beforeTake(), pivot.init()
        );
        Action PlaceFromGround = new SequentialAction(
                clawTilt.take(), claw.close(), clawRotate.horizontal(), clawTilt.place(), linkage.place()
        );
        
        
        Action autoSequence = new SequentialAction(
                TakeFromGround,
                new SleepAction(2),
                PlaceFromGround
                );

        Action pid = new ParallelAction(
                lift.update()
        );

        waitForStart();
        Actions.runBlocking(
                new ParallelAction(
                        autoSequence, pid
                )
        );
    }
}
