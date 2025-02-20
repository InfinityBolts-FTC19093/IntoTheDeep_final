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
                .strafeTo(new Vector2d(8, -32), null, new ProfileAccelConstraint(-70, 70));

        TrajectoryActionBuilder HUMAN = PRELOAD.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(33, -37, Math.toRadians(270)), Math.toRadians(0),null, new ProfileAccelConstraint(-70, 70))
                .splineToLinearHeading(new Pose2d(38, -10, Math.toRadians(270)), Math.toRadians(0),null, new ProfileAccelConstraint(-70, 70))
                .splineToLinearHeading(new Pose2d(50, -10, Math.toRadians(270)), Math.toRadians(0),null, new ProfileAccelConstraint(-70, 70));


//        TrajectoryActionBuilder GROUND1 = PRELOAD.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(27, -34), Math.toRadians(26));
//
//        TrajectoryActionBuilder THROW1 = GROUND1.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(34, -37), Math.toRadians(310));
//
//        TrajectoryActionBuilder GROUND2 = THROW1.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(37, -34), Math.toRadians(38));
//
//        TrajectoryActionBuilder THROW2 = GROUND2.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d( 39, -37), Math.toRadians(310));
//
//        TrajectoryActionBuilder GROUND3 = THROW2.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(47, -34) ,Math.toRadians(37));
//
//        TrajectoryActionBuilder THROW3 = GROUND3.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(46, -35), Math.toRadians(300));

        TrajectoryActionBuilder GTS1 = HUMAN.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(50, -50), Math.toRadians(90))
                .strafeTo(new Vector2d(50, -58));

        TrajectoryActionBuilder PLACE1 = GTS1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(4, -35), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(4, -32));

        TrajectoryActionBuilder GTS2 = PLACE1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(50, -50), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(50, -56));

        TrajectoryActionBuilder PLACE2 = GTS2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(6, -35), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(6, -32));

        TrajectoryActionBuilder GTS3 = PLACE2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(50, -50), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(50, -56));

        TrajectoryActionBuilder PLACE3 = GTS3.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(8, -35), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(8, -32));

        TrajectoryActionBuilder GTS4 = PLACE3.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(50, -50), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(50, -56));

        TrajectoryActionBuilder PLACE4 = GTS4.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(10, -35), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(10, -32));


        Action PreLoad = new SequentialAction(
                lift.liftChamber(), sliderTilt.chamber(), new SleepAction(.1), turret.place()
        );

        Action PlacePreload= new SequentialAction(
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


        Action Place1= new SequentialAction(
                lift.liftPlace(), new SleepAction(.2), sliderClaw.open()
        );

        Action Place2= new SequentialAction(
                lift.liftPlace(), new SleepAction(.2), sliderClaw.open()
        );

        Action Place3= new SequentialAction(
                lift.liftPlace(), new SleepAction(.2), sliderClaw.open()
        );

        Action Place4= new SequentialAction(
                lift.liftPlace(), new SleepAction(.2), sliderClaw.open()
        );

        Action Human1 = new SequentialAction(
                sliderTilt.human(), new SleepAction(.2), lift.liftDown()
        );

        Action Human2 = new SequentialAction(
                sliderTilt.human(), new SleepAction(.2), lift.liftDown()
        );

        Action Human3 = new SequentialAction(
                sliderTilt.human(), new SleepAction(.2), lift.liftDown()
        );

        Action Human4 = new SequentialAction(
                sliderTilt.human(), new SleepAction(.2), lift.liftDown()
        );

        Action Chamber2 = new SequentialAction(
                sliderClaw.close(), new SleepAction(.2), lift.liftChamber(), sliderTilt.chamber()
        );

        Action Chamber3 = new SequentialAction(
                sliderClaw.close(), new SleepAction(.2), lift.liftChamber(), sliderTilt.chamber()
        );

        Action Chamber4 = new SequentialAction(
                sliderClaw.close(), new SleepAction(.2), lift.liftChamber(), sliderTilt.chamber()
        );


        Action safepose = safePose.build();
        Action preloadAction = PRELOAD.build();
        Action humanAction = HUMAN.build();
//        Action g1 = GROUND1.build();
//        Action t1 = THROW1.build();
//        Action g2 = GROUND2.build();
//        Action t2 = THROW2.build();
//        Action g3 = GROUND3.build();
//        Action t3 = THROW3.build();
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
            new SleepAction(.2),
            sliderClaw.open(),
            Human,
//            g1,
//            TakeFromGround,
//            new SleepAction(.3),
//            t1,
//            claw.open(),
//            clawTilt.beforeTake(),
//            g2,
//            TakeFromGround2,
//            new SleepAction(.3),
//            t2,
//            claw.open(),
//            clawTilt.beforeTake(),
//                g3,
//                TakeFromGround3,
//                new SleepAction(.3),
//                t3,
//                claw.open(),
//                new SleepAction(.2),
//                retractLinkage,

                humanAction,

                GTS1Action
//                Chamber,
//                new SleepAction(.1),
//                chamber1,
//                new SleepAction(.1),
//                Place1,
//                new SleepAction(.1),
//                Human1,
//                GTS2Action,
//                Chamber2,
//                new SleepAction(.1),
//                chamber2,
//                new SleepAction(.1),
//                Place2,
//                new SleepAction(.1),
//                Human2,
//                GTS3Action,
//                Chamber3,
//                new SleepAction(.1),
//                chamber3,
//                new SleepAction(.1),
//                Place3,
//                new SleepAction(.1),
//                Human3,
//                GTS3Action,
//                Chamber4,
//                new SleepAction(.1),
//                chamber4,
//                new SleepAction(.1),
//                Place4,
//                new SleepAction(.1),
//                Human4

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
