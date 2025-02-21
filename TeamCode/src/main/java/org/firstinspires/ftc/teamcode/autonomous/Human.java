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
import org.firstinspires.ftc.teamcode.autonomous.actions.actionsManual;
import org.firstinspires.ftc.teamcode.constants.RobotMap;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

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
        actionsManual.Claw claw = new actionsManual.Claw(hardwareMap);
        actionsManual.Linkage linkage = new actionsManual.Linkage(hardwareMap);
        actionsManual.SliderTilt sliderTilt = new actionsManual.SliderTilt(hardwareMap);
        actionsManual.Turret turret = new actionsManual.Turret(hardwareMap);
        actionsManual.Update updateAuto = new actionsManual.Update(hardwareMap);
        actionsManual.ClawTilt clawTilt = new actionsManual.ClawTilt(hardwareMap);

        TrajectoryActionBuilder safePose = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(9, -61.4 ));

        TrajectoryActionBuilder PRELOAD = safePose.endTrajectory().fresh()
                .strafeTo(new Vector2d(8, -32), null, new ProfileAccelConstraint(-70, 70));

        TrajectoryActionBuilder HUMAN = PRELOAD.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(32, -40), Math.toRadians(90), null, new ProfileAccelConstraint(-60, 60))
                .splineToLinearHeading(new Pose2d(32, -14, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(50, -14, Math.toRadians(90)), Math.toRadians(90))

                .strafeToLinearHeading(new Vector2d(50, -48), Math.toRadians(90), null, new ProfileAccelConstraint(-90, 90))

                .strafeToLinearHeading(new Vector2d(50, -17), Math.toRadians(90), null, new ProfileAccelConstraint(-90, 90))
                .strafeToLinearHeading(new Vector2d(62, -17), Math.toRadians(90), null, new ProfileAccelConstraint(-90, 90))

                .strafeToLinearHeading(new Vector2d(60, -48), Math.toRadians(90), null, new ProfileAccelConstraint(-90, 90))

                .strafeToLinearHeading(new Vector2d(60, -17), Math.toRadians(90), null, new ProfileAccelConstraint(-90, 90))
                .strafeToLinearHeading(new Vector2d(70, -17), Math.toRadians(90), null, new ProfileAccelConstraint(-90, 90));


        TrajectoryActionBuilder GTS1 = HUMAN.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(64, -51), Math.toRadians(90), null, new ProfileAccelConstraint(-120, 120));

        TrajectoryActionBuilder PLACE1 = GTS1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(9, -36), Math.toRadians(90), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(9, -31.5));

        TrajectoryActionBuilder GTS2 = PLACE1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(50, -50), Math.toRadians(90), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(50, -52));

        TrajectoryActionBuilder PLACE2 = GTS2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(11, -36), Math.toRadians(90), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(11, -31.5));

        TrajectoryActionBuilder GTS3 = PLACE2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(50, -50), Math.toRadians(90), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(50, -53));

        TrajectoryActionBuilder PLACE3 = GTS3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(13, -36), Math.toRadians(90), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(13, -31.5));

        TrajectoryActionBuilder GTS4 = PLACE3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(50, -50), Math.toRadians(90), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(50, -54));

        TrajectoryActionBuilder PLACE4 = GTS4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(15, -35), Math.toRadians(90), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(15, -31.5));

        TrajectoryActionBuilder PARK = PLACE4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(13, -55), Math.toRadians(0), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(45, -55));

        Action PreLoad = new SequentialAction(
                lift.liftChamber(), sliderTilt.chamber(), new SleepAction(.1), turret.place()
        );

        Action PlacePreload= new SequentialAction(
                lift.liftPlace(), new SleepAction(.2), sliderClaw.open()
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

        Action Human = new SequentialAction(
                sliderTilt.human(), new SleepAction(.2), lift.liftDown()
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

        Action Chamber1 = new SequentialAction(
                sliderClaw.close(), new SleepAction(.2), lift.liftChamber(), sliderTilt.chamber()
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

        Action Park = new SequentialAction(
                linkage.take(), clawTilt.beforeTake()
        );

        Action safepose = safePose.build();
        Action preloadAction = PRELOAD.build();
        Action humanAction = HUMAN.build();
        Action GTS1Action = GTS1.build();
        Action chamber1 = PLACE1.build();
        Action GTS2Action = GTS2.build();
        Action chamber2 = PLACE2.build();
        Action GTS3Action = GTS3.build();
        Action chamber3 = PLACE3.build();
        Action GTS4Action = GTS4.build();
        Action chamber4 = PLACE4.build();
        Action parkAction = PARK.build();


        Action autoSequence = new SequentialAction(
                safepose,
                PreLoad,
                preloadAction,
                PlacePreload,
                sliderClaw.open(), //a pus preloadu
                Human,
                humanAction,  //duce sample urile
                GTS1Action, //merge la human
                Chamber1,
                chamber1,
                Place1,             //a pus al doilea
                Human1,
                GTS2Action,
                Chamber2,
                chamber2,
                Place2,              //a pus al treilea
                Human2,
                GTS3Action,
                Chamber3,
                chamber3,
                Place3,             //a pus al patrulea
                Human3,
                GTS4Action,
                Chamber4,
                chamber4,
                Place4,             //a pus al cincilea
                lift.liftDown(),
                sliderTilt.human(),
                parkAction
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
