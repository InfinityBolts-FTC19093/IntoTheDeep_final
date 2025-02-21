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

@Autonomous (name = "Basket", group = "#")
public class Basket extends LinearOpMode {
    RobotMap robot;
    slider_controller sliderController;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotMap(hardwareMap);
        Pose2d startPose = new Pose2d(-33, -61.5, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        sliderController = new slider_controller(robot.slider);

//        SliderAction     = new PrepareAuto(robot.slider_claw, robot.slider_claw_tilt, robot.turret, robot.slider, robot.claw);
//        LinkageAction    = new CollectAuto(robot.claw ,robot.claw_tilt, robot.linkage, robot.claw_rotate, robot.claw_pivot);
//        ScoreAction      = new ScoreAuto(robot.claw, robot.claw_tilt, robot.linkage, robot.claw_rotate, robot.claw_pivot, robot.slider_claw, robot.slider_claw_tilt, robot.turret, robot.slider, LinkageAction, SliderAction);

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
                .strafeTo(new Vector2d(-33.1, -61.5));

        TrajectoryActionBuilder PRELOAD = safePose.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(190));

        TrajectoryActionBuilder G1 = PRELOAD.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-45, -47), Math.toRadians(85), null, new ProfileAccelConstraint(-40, 40));

        TrajectoryActionBuilder PLACE1 = G1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-53, -52), Math.toRadians(65));

        TrajectoryActionBuilder G2 = PLACE1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-59, -47), Math.toRadians(95), null, new ProfileAccelConstraint(-40, 40));

        TrajectoryActionBuilder PLACE2 = G1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-53, -52), Math.toRadians(65));

        TrajectoryActionBuilder G3 = PLACE2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-59, -44), Math.toRadians(135), null, new ProfileAccelConstraint(-40, 40));

        TrajectoryActionBuilder PLACE3 = G3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-53, -52), Math.toRadians(55));

        TrajectoryActionBuilder PARK = PLACE3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-35, -10), Math.toRadians(0))
                .strafeTo(new Vector2d(-20, -10));



        Action TakeFromGround = new SequentialAction(
                linkage.take(), claw.open(), clawTilt.beforeTake()
        );
        Action PlaceInSlider = new SequentialAction(
                clawTilt.take(), new SleepAction(.3), claw.close(), new SleepAction(.3), clawRotate.horizontal(), clawTilt.place(),new SleepAction(.2), linkage.place(), new SleepAction(.2),sliderClaw.open(),new SleepAction(.2), turret.take(),new SleepAction(.2), sliderTilt.take(),new SleepAction(.2), lift.takeFromLinkage(),new SleepAction(.2), claw.open(), new SleepAction(.2), sliderClaw.close()
        );

        Action BeforeTakeFromLinkage = new SequentialAction(
                lift.beforeLinkage(), new SleepAction(.05), sliderTilt.beforeLinkage()
        );
        Action PlaceOnBasket = new SequentialAction(
                sliderClaw.close(), new SleepAction(.2), lift.liftBasket(), new SleepAction(.3), sliderTilt.basket() ,turret.Basket()
        );
        Action PlacePreload = new SequentialAction(
                sliderClaw.close(), new SleepAction(.1), lift.liftPreload(), new SleepAction(.2), sliderTilt.chamber(), turret.place()
        );


        Action TakeFromGround2 = new SequentialAction(
                linkage.take(), claw.open(), clawTilt.beforeTake()
        );
        Action PlaceInSlider2 = new SequentialAction(
                clawTilt.take(), new SleepAction(.3), claw.close(), new SleepAction(.3), clawRotate.horizontal(), clawTilt.place(),new SleepAction(.2), linkage.place(), new SleepAction(.2),sliderClaw.open(),new SleepAction(.2), turret.take(),new SleepAction(.2), sliderTilt.take(),new SleepAction(.2), lift.takeFromLinkage(),new SleepAction(.2), claw.open(), new SleepAction(.2), sliderClaw.close()
        );

        Action BeforeTakeFromLinkage2 = new SequentialAction(
                lift.beforeLinkage(), new SleepAction(.05), sliderTilt.beforeLinkage()
        );
        Action PlaceOnBasket2 = new SequentialAction(
                sliderClaw.close(), new SleepAction(.2), lift.liftBasket(), new SleepAction(.3), sliderTilt.basket() ,turret.Basket()
        );


        Action TakeFromGround3 = new SequentialAction(
                linkage.take(), claw.open(), clawTilt.beforeTake(), pivot.take()
        );
        Action PlaceInSlider3 = new SequentialAction(
                clawTilt.take(), new SleepAction(.3), claw.close(), new SleepAction(.1), pivot.init(), new SleepAction(.3), clawRotate.horizontal(), clawTilt.place(),new SleepAction(.2), linkage.place(), new SleepAction(.2),sliderClaw.open(),new SleepAction(.2), turret.take(),new SleepAction(.2), sliderTilt.take(),new SleepAction(.2), lift.takeFromLinkage(),new SleepAction(.2), claw.open(), new SleepAction(.2), sliderClaw.close()
        );

        Action BeforeTakeFromLinkage3 = new SequentialAction(
                lift.beforeLinkage(), new SleepAction(.05), sliderTilt.beforeLinkage()
        );
        Action PlaceOnBasket3 = new SequentialAction(
                sliderClaw.close(), new SleepAction(.2), lift.liftBasket(), new SleepAction(.3), sliderTilt.basket() ,turret.Basket()
        );


        Action safepose = safePose.build();
        Action preload = PRELOAD.build();
        Action g1 = G1.build();
        Action place1 = PLACE1.build();
        Action g2 = G2.build();
        Action place2 = PLACE2.build();
        Action g3 = G3.build();
        Action place3 = PLACE3.build();
        Action park = PARK.build();

        Action autoSequence = new SequentialAction(
                safepose,
                PlacePreload,
                preload,
                new SleepAction(.4),
                sliderClaw.open(),
                new SleepAction(.2),
                g1,
                BeforeTakeFromLinkage,
                new SleepAction(.4),
                TakeFromGround,
                new SleepAction(.5),
                PlaceInSlider,
                new SleepAction(.5),
                PlaceOnBasket,
                new SleepAction(.7),
                place1,
                new SleepAction(.6),
                sliderClaw.open(),
                g2,
                BeforeTakeFromLinkage2,
                new SleepAction(.7),
                TakeFromGround2,
                new SleepAction(.5),
                PlaceInSlider2,
                new SleepAction(.5),
                PlaceOnBasket2,
                new SleepAction(.7),
                place2,
                new SleepAction(.6),
                sliderClaw.open(),
                g3,
                BeforeTakeFromLinkage3,
                new SleepAction(.7),
                TakeFromGround3,
                new SleepAction(.5),
                PlaceInSlider3,
                new SleepAction(.5),
                PlaceOnBasket3,
                new SleepAction(.7),
                place3,
                new SleepAction(.6),
                sliderClaw.open(),
                new SleepAction(.2),
                sliderTilt.park(),
                lift.liftDown(),
                park
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
