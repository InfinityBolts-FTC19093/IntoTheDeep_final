package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MECANUM.MecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.actions.actions;
import org.firstinspires.ftc.teamcode.constants.RobotMap;


@Autonomous (name = "TEST")
public class AutoTestDetectie extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotMap robot = new RobotMap(hardwareMap);
        Pose2d startPose = new Pose2d(9, -61.5, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        actions.Auto auto = new actions.Auto(hardwareMap);

        if (!isStarted()) {
            auto.initAll();
        }

        Action autoSequence = new SequentialAction(
            auto.Basket()
        );

        Action pid = new ParallelAction(
          auto.updateAll()
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
