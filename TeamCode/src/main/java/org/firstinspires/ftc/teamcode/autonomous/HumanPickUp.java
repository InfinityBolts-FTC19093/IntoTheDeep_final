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
                .strafeTo(new Vector2d(4, -30.7523413412342356262), null, new ProfileAccelConstraint(-70, 70));

        TrajectoryActionBuilder GROUND1 = PRELOAD.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(29.5, -36), Math.toRadians(30));

        TrajectoryActionBuilder DROP1 = GROUND1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35, -40), Math.toRadians(310), null, new ProfileAccelConstraint(-120, 120));

        TrajectoryActionBuilder GROUND2 = DROP1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(42, -39), Math.toRadians(50), null, new ProfileAccelConstraint(-120, 120));

        TrajectoryActionBuilder DROP2 = GROUND1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(42, -40), Math.toRadians(300), null, new ProfileAccelConstraint(-120, 120));

        TrajectoryActionBuilder GROUND3 = DROP2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(50, -38), Math.toRadians(50), null, new ProfileAccelConstraint(-120, 120));

        TrajectoryActionBuilder DROP3 = GROUND1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(50, -37), Math.toRadians(280), null, new ProfileAccelConstraint(-120, 120));

        TrajectoryActionBuilder GTS1 = DROP3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(40, -50), Math.toRadians(90), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(40, -56));

        TrajectoryActionBuilder PLACE1 = GTS1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(5.5, -28.5), Math.toRadians(90), null, new ProfileAccelConstraint(-120, 120));

        TrajectoryActionBuilder GTS2 = PLACE1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(40, -50), Math.toRadians(90), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(40, -55));

        TrajectoryActionBuilder PLACE2 = GTS2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(7, -28.5), Math.toRadians(90), null, new ProfileAccelConstraint(-120, 120));

        TrajectoryActionBuilder GTS3 = PLACE2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(40, -50), Math.toRadians(90), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(40, -55));

        TrajectoryActionBuilder PLACE3 = GTS3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(8.5, -30), Math.toRadians(90), null, new ProfileAccelConstraint(-120, 120));

        TrajectoryActionBuilder GTS4 = PLACE3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(40, -50), Math.toRadians(90), null, new ProfileAccelConstraint(-120, 120))
                .strafeTo(new Vector2d(40, -54.5));

        TrajectoryActionBuilder PLACE4 = GTS4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(10, -28.5), Math.toRadians(90), null, new ProfileAccelConstraint(-120, 120));

        TrajectoryActionBuilder GTS5 = PLACE4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(29.5, -59), Math.toRadians(355));

        TrajectoryActionBuilder Basket = GTS5.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-48, -55), Math.toRadians(15));

        TrajectoryActionBuilder PARK = Basket.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(30, -60), Math.toRadians(350), null, new ProfileAccelConstraint(-200, 200));

        Action PreLoad = new SequentialAction(
                lift.liftChamber(), sliderTilt.chamber(), new SleepAction(.1), turret.place()
        );

        Action PlacePreload= new SequentialAction(
                lift.liftPlace(), new SleepAction(.35), sliderClaw.open()
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
                clawTilt.take(), new SleepAction(.2), claw.close(), new SleepAction(.1), clawTilt.beforeTake()
        );

        Action TakeGround2 = new SequentialAction(
                clawTilt.take(), new SleepAction(.2), claw.close(), new SleepAction(.1), clawTilt.beforeTake()
        );

        Action TakeGround3 = new SequentialAction(
                clawTilt.take(), new SleepAction(.2), claw.close(), new SleepAction(.1), clawTilt.beforeTake()
        );

        Action Retract = new SequentialAction(
                linkage.place(), clawTilt.init(), claw.open(), rotate.horizontal(), pivot.init()
        );

        Action BeforeTakeSample = new SequentialAction(
                linkage.take(), clawTilt.beforeTake(), rotate.horizontal(), pivot.init(), claw.open()
        );

        Action TakeSample = new SequentialAction(
                clawTilt.take(), new SleepAction(.35), claw.close(), new SleepAction(.1), linkage.place(), clawTilt.place(), new SleepAction(.1)
        );

        Action Transfer = new SequentialAction(
                sliderTilt.take() , new SleepAction(.3), sliderClaw.close() , new SleepAction(.05), claw.open(), new SleepAction(.05), lift.liftBasket(), sliderTilt.basket()
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
        Action GTS5Action = GTS5.build();
        Action basketAction = Basket.build();
        Action parkAction = PARK.build();

        Action autoSequence = new SequentialAction(
                safepose,
                PreLoad,
                preloadAction,
                PlacePreload,
                sliderClaw.open(), //a pus preloadu
                Human,
                GR1,
                BeforeGround,
                new SleepAction(.4),
                TakeGround1,
                drop1,
                claw.open(),
                new SleepAction(.1),
                GR2,
                new SleepAction(.2),
                TakeGround2,
                drop2,
                claw.open(),
                new SleepAction(.1),
                GR3,
                new SleepAction(.2),
                TakeGround3,
                drop3,
                claw.open(),
                new SleepAction(.1),
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
                lift.takeFromLinkage(),
                GTS5Action,
                sliderTilt.beforeLinkage(),
                BeforeTakeSample,
                new SleepAction(.2),
                TakeSample,
                new SleepAction(.3),
                Transfer,
                basketAction,
                new SleepAction(.1),
                sliderClaw.open(),
                new SleepAction(.15),
                sliderTilt.park(),
                lift.liftDown(),
                linkage.take(),
                parkAction
        );

        Action pid = new ParallelAction(
                lift.update()
        );

        if (!isStarted()) {
            updateAuto.initAllChamber();
        }

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        autoSequence, pid
                )
        );
    }
}
