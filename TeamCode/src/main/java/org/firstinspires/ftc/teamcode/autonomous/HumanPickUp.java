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

import org.firstinspires.ftc.teamcode.MECANUM.PinpointDrive;
import org.firstinspires.ftc.teamcode.autonomous.actions.actionsManual;
import org.firstinspires.ftc.teamcode.constants.RobotMap;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

@Autonomous (name = "Human2.0", group = "#")
public class HumanPickUp extends LinearOpMode {

    slider_controller sliderController;
    @Override
    public void runOpMode() {
        RobotMap robot      = new RobotMap(hardwareMap);
        Pose2d startPose    = new Pose2d(9, -61.5, Math.toRadians(90));
        PinpointDrive drive  = new PinpointDrive(hardwareMap, startPose);

        sliderController = new slider_controller(robot.slider, hardwareMap);

        actionsManual.Lift lift = new actionsManual.Lift(hardwareMap);
        actionsManual.SliderClaw sliderClaw = new actionsManual.SliderClaw(hardwareMap);
        actionsManual.Claw claw = new actionsManual.Claw(hardwareMap);
        actionsManual.Linkage linkage = new actionsManual.Linkage(hardwareMap);
        actionsManual.SliderTilt sliderTilt = new actionsManual.SliderTilt(hardwareMap);
        actionsManual.Turret turret = new actionsManual.Turret(hardwareMap);
        actionsManual.Update updateAuto = new actionsManual.Update(hardwareMap);
        actionsManual.ClawTilt clawTilt = new actionsManual.ClawTilt(hardwareMap);
        actionsManual.Pivot pivot = new actionsManual.Pivot(hardwareMap);
        actionsManual.ClawRotate rotate = new actionsManual.ClawRotate(hardwareMap);

        TrajectoryActionBuilder safePose = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(9, -61.4 ));

        TrajectoryActionBuilder PRELOAD = safePose.endTrajectory().fresh()
                .strafeTo(new Vector2d(6, -30), null, new ProfileAccelConstraint(-70, 70));

        TrajectoryActionBuilder GROUND1 = PRELOAD.endTrajectory().fresh()
                .strafeTo(new Vector2d(8, -35))
                .splineToLinearHeading(new Pose2d(33, -36, Math.toRadians(45)), Math.toRadians(90));

        TrajectoryActionBuilder DROP1 = GROUND1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(30, -40), Math.toRadians(300));

        TrajectoryActionBuilder GROUND2 = DROP1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(38, -38), Math.toRadians(45));

        TrajectoryActionBuilder DROP2 = GROUND1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(38, -40), Math.toRadians(300));

        TrajectoryActionBuilder GROUND3 = DROP2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(46, -38), Math.toRadians(35));

        TrajectoryActionBuilder DROP3 = GROUND1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(46, -40), Math.toRadians(300));

        TrajectoryActionBuilder GO_TO_STACK1 = DROP3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(40, -55), Math.toRadians(90));

        TrajectoryActionBuilder GTS1 = GO_TO_STACK1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(65, -52.8), Math.toRadians(90), null, new ProfileAccelConstraint(-90, 90));

        TrajectoryActionBuilder PLACE1 = GTS1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(6, -38), Math.toRadians(90), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(6, -30));

        TrajectoryActionBuilder GTS2 = PLACE1.endTrajectory().fresh()
                .strafeTo(new Vector2d(6, -34))

                .strafeToLinearHeading(new Vector2d(45, -50), Math.toRadians(90), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(45, -55));

        TrajectoryActionBuilder PLACE2 = GTS2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(7.5, -38), Math.toRadians(90), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(7.5, -33.5));

        TrajectoryActionBuilder GTS3 = PLACE2.endTrajectory().fresh()
                .strafeTo(new Vector2d(10, -36))

                .strafeToLinearHeading(new Vector2d(45, -50), Math.toRadians(90), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(45, -56));

        TrajectoryActionBuilder PLACE3 = GTS3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(11, -38), Math.toRadians(90), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(11, -32));

        TrajectoryActionBuilder GTS4 = PLACE3.endTrajectory().fresh()
                .strafeTo(new Vector2d(11, -35.5))

                .strafeToLinearHeading(new Vector2d(45, -50), Math.toRadians(90), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(45, -56));

        TrajectoryActionBuilder PLACE4 = GTS4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(14.5, -38), Math.toRadians(90), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(14.5, -30));

        TrajectoryActionBuilder PARK = PLACE4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35, -45), Math.toRadians(320));

        Action PreLoad = new SequentialAction(
                lift.liftChamber(), sliderTilt.chamber(), new SleepAction(.1), turret.place()
        );

        Action PlacePreload= new SequentialAction(
                lift.liftPlace(), new SleepAction(.2), sliderClaw.open()
        );

        Action Place1= new SequentialAction(
                lift.liftPlace(), new SleepAction(.3), sliderClaw.open()
        );

        Action Place2= new SequentialAction(
                lift.liftPlace(), new SleepAction(.3), sliderClaw.open()
        );

        Action Place3= new SequentialAction(
                lift.liftPlace(), new SleepAction(.3), sliderClaw.open()
        );

        Action Place4= new SequentialAction(
                lift.liftPlace(), new SleepAction(.3), sliderClaw.open()
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

        Action BeforeGround = new SequentialAction(
                linkage.take(), claw.open(), clawTilt.beforeTake(), pivot.take(), rotate.vertical()
        );

        Action TakeGround1 = new SequentialAction(
            clawTilt.take(), new SleepAction(.1), claw.close(), new SleepAction(.1), clawTilt.beforeTake()
        );

        Action TakeGround2 = new SequentialAction(
                clawTilt.take(), new SleepAction(.1), claw.close(), new SleepAction(.1), clawTilt.beforeTake()
        );

        Action TakeGround3 = new SequentialAction(
                clawTilt.take(), new SleepAction(.1), claw.close(), new SleepAction(.1), clawTilt.beforeTake()
        );

        Action Retract = new SequentialAction(
                linkage.place(), clawTilt.init(), claw.open(), rotate.horizontal(), pivot.init()
        );

        Action safepose = safePose.build();
        Action preloadAction = PRELOAD.build();
        Action GR1 = GROUND1.build();
        Action drop1 = DROP1.build();
        Action GR2 = GROUND2.build();
        Action drop2 = DROP2.build();
        Action GR3 = GROUND3.build();
        Action drop3 = DROP3.build();
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
                GR1,
                new SleepAction(.3),
                BeforeGround,
                new SleepAction(.1),
                TakeGround1,
                drop1,
                claw.open(),
                GR2,
                new SleepAction(.1),
                TakeGround2,
                drop2,
                claw.open(),
                GR3,
                new SleepAction(.1),
                TakeGround3,
                drop3,
                claw.open(),
                Retract,            //duce sample urile
                GTS1Action,
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
