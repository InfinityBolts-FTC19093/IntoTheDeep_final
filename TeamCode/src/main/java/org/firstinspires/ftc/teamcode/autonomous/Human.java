package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MECANUM.MecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.actions.CollectAuto;
import org.firstinspires.ftc.teamcode.autonomous.actions.PrepareAuto;
import org.firstinspires.ftc.teamcode.autonomous.actions.ScoreAuto;
import org.firstinspires.ftc.teamcode.autonomous.actions.actions;
import org.firstinspires.ftc.teamcode.autonomous.actions.actionsManual;
import org.firstinspires.ftc.teamcode.constants.RobotMap;
import org.firstinspires.ftc.teamcode.systems.slider_controller;
import org.opencv.core.Mat;

@Autonomous (name = "Human", group = "#")
public class Human extends LinearOpMode {
    RobotMap robot;
    slider_controller sliderController;
    @Override
    public void runOpMode() {
        RobotMap robot      = new RobotMap(hardwareMap);
        Pose2d startPose    = new Pose2d(9, -61.5, Math.toRadians(90));
        MecanumDrive drive  = new MecanumDrive(hardwareMap, startPose);

        sliderController = new slider_controller(robot.slider);

        actionsManual.Lift lift = new actionsManual.Lift(hardwareMap);
        actionsManual.SliderClaw sliderClaw = new actionsManual.SliderClaw(hardwareMap);
        actionsManual.Linkage linkage = new actionsManual.Linkage(hardwareMap);
        actionsManual.Claw claw = new actionsManual.Claw(hardwareMap);
        actionsManual.ClawRotate clawRotate = new actionsManual.ClawRotate(hardwareMap);
        actionsManual.SliderTilt sliderTilt = new actionsManual.SliderTilt(hardwareMap);
        actionsManual.Turret turret = new actionsManual.Turret(hardwareMap);
        actionsManual.ClawTilt clawTilt = new actionsManual.ClawTilt(hardwareMap);
        actionsManual.Pivot pivot = new actionsManual.Pivot(hardwareMap);
        actionsManual.Update updateAuto = new actionsManual.Update(hardwareMap);

        TrajectoryActionBuilder safePose = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(9, -61.4 ));

        TrajectoryActionBuilder PRELOAD = safePose.endTrajectory().fresh()
                .strafeTo(new Vector2d(8, -33), null, new ProfileAccelConstraint(-70, 70));

        TrajectoryActionBuilder GROUND1 = PRELOAD.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(28.5, -34), Math.toRadians(27));

        TrajectoryActionBuilder THROW1 = GROUND1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(34, -37), Math.toRadians(310));

        TrajectoryActionBuilder GROUND2 = THROW1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(39, -34), Math.toRadians(39));

        TrajectoryActionBuilder THROW2 = GROUND2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d( 39, -37), Math.toRadians(310));

        TrajectoryActionBuilder GROUND3 = THROW2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(46, -34) ,Math.toRadians(45));

        TrajectoryActionBuilder THROW3 = GROUND3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(46, -35), Math.toRadians(145));

        TrajectoryActionBuilder GTS1 = THROW3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(50, -55), Math.toRadians(90));

        TrajectoryActionBuilder PLACE1 = GTS1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-4, -35), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(-4, -34));

        TrajectoryActionBuilder GTS2 = PLACE1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(45, -55), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(45, -61));

        TrajectoryActionBuilder PLACE2 = GTS2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(0, -35), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(0, -32));

        TrajectoryActionBuilder GTS3 = PLACE2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(45, -55), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(45, -61));

        TrajectoryActionBuilder PLACE3 = GTS3.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(4, -35), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(4, -32));

        TrajectoryActionBuilder GTS4 = PLACE3.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(45, -55), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(45, -61));

        TrajectoryActionBuilder PLACE4 = GTS4.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(8, -35), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(8, -32));


        Action PreLoad = new SequentialAction(
                lift.liftChamber(), sliderTilt.chamber(), new SleepAction(.1), turret.place()
        );

        Action Place= new SequentialAction(
                 lift.liftPlace(), new SleepAction(.2), sliderClaw.open()
        );

        Action Human = new SequentialAction(
                sliderTilt.human(), new SleepAction(.2), lift.liftDown()
        );

        Action Chamber = new SequentialAction(
                sliderClaw.close(), new SleepAction(.2), lift.liftChamber(), sliderTilt.chamber()
        );

        Action BeforeTakeFromLinkage = new SequentialAction(
                lift.beforeLinkage(), new SleepAction(.05), sliderTilt.beforeLinkage()
        );

        Action TakeFromGround = new SequentialAction(
                claw.open() ,linkage.take(), clawTilt.take(), pivot.take(), clawRotate.vertical(), new SleepAction(.5), claw.close(), new SleepAction(.2), clawTilt.beforeTake()
        );

        Action ThrowHuman = new SequentialAction(
                linkage.take(), clawTilt.place(), new SleepAction(.1), claw.open(), new SleepAction(.05), linkage.place(), clawTilt.beforeTake()
        );

        Action retractLinkage = new SequentialAction(
              claw.close(), clawTilt.place(), linkage.place(), clawRotate.horizontal(), pivot.init()
        );

        Action TakeFromGround2 = new SequentialAction(
                claw.open() ,linkage.take(), clawTilt.take(), pivot.take(), clawRotate.vertical(), new SleepAction(.5), claw.close(), new SleepAction(.2), clawTilt.beforeTake()
        );

        Action TakeFromGround3 = new SequentialAction(
                claw.open() ,linkage.take(), clawTilt.take(), pivot.take(), clawRotate.vertical(), new SleepAction(.5), claw.close(), new SleepAction(.2), clawTilt.beforeTake()
        );


        Action safepose = safePose.build();
        Action preloadAction = PRELOAD.build();
        Action g1 = GROUND1.build();
        Action t1 = THROW1.build();
        Action g2 = GROUND2.build();
        Action t2 = THROW2.build();
        Action g3 = GROUND3.build();
        Action t3 = THROW3.build();
        Action GTS1Action = GTS1.build();
        Action chamber1 = PLACE1.build();
        Action GTS2Action = GTS2.build();
        Action chamber2 = PLACE2.build();
        Action GTS3Action = GTS3.build();
        Action chamber3 = PLACE3.build();
        Action GTS4Action = GTS4.build();
        Action chamber4 = PLACE4.build();


        Action autoSequence = new SequentialAction(
            safepose,
            PreLoad,
            preloadAction,
            new SleepAction(.4),
            Place,
            new SleepAction(.2),
            Human,
            g1,
            TakeFromGround,
            new SleepAction(.7),
            t1,
            claw.open(),
            clawTilt.beforeTake(),
            g2,
            TakeFromGround2,
            t2,
            claw.open(),
            clawTilt.beforeTake()
        );

        Action pid = new ParallelAction(
          lift.update()
        );

        if (!isStarted()) {
            updateAuto.initAll();
        }

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        autoSequence, pid
                )
        );
    }
}
