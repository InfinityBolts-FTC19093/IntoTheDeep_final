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
import org.firstinspires.ftc.teamcode.MECANUM.PinpointDrive;
import org.firstinspires.ftc.teamcode.autonomous.actions.actionsManual;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.RobotMap;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

@Autonomous (name = "Basket", group = "#")
public class Basket extends LinearOpMode {
    RobotMap robot;
    slider_controller sliderController;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotMap(hardwareMap);
        Pose2d startPose = new Pose2d(-33, -61.5, Math.toRadians(0));
        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);

        sliderController = new slider_controller(robot.slider, hardwareMap);


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
                .strafeToLinearHeading(new Vector2d(-54, -53), Math.toRadians(30));

        TrajectoryActionBuilder G1 = PRELOAD.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-58, -47), Math.toRadians(90), null, new ProfileAccelConstraint(-70, 70));

        TrajectoryActionBuilder PLACE1 = G1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-58, -50), Math.toRadians(65));

        TrajectoryActionBuilder G2 = PLACE1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-64.5, -44), Math.toRadians(90), null, new ProfileAccelConstraint(-70, 70));

        TrajectoryActionBuilder PLACE2 = G1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-64, -50), Math.toRadians(65));

        TrajectoryActionBuilder G3 = G2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-59, -42), Math.toRadians(135), null, new ProfileAccelConstraint(-70, 70));

        TrajectoryActionBuilder PLACE3 = G3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-59, -50), Math.toRadians(65));

        TrajectoryActionBuilder PARK = PLACE3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-35, -10), Math.toRadians(0))
                .strafeTo(new Vector2d(-20, -10));

        Action TakeFromGround = new SequentialAction(
                linkage.take(), claw.open(), clawTilt.beforeTake(), clawRotate.horizontal()
        );
        Action PlaceInSlider = new SequentialAction(
                clawTilt.take(), new SleepAction(.2), claw.close(), new SleepAction(.2), clawRotate.inverted(), clawTilt.place(),new SleepAction(.2), linkage.place(), new SleepAction(.2),sliderClaw.open(), turret.take(),new SleepAction(.1), sliderTilt.take(),new SleepAction(.2), lift.takeFromLinkage(),new SleepAction(.2), claw.open(), sliderClaw.close()
        );

        Action BeforeTakeFromLinkage = new SequentialAction(
                lift.beforeLinkage(), new SleepAction(.05), sliderTilt.beforeLinkage()
        );
        Action PlaceOnBasket = new SequentialAction(
                sliderClaw.close(), new SleepAction(.2), lift.liftBasket(), new SleepAction(.3), sliderTilt.basket() ,turret.place()
        );
        Action PlacePreload = new SequentialAction(
                sliderClaw.close(), new SleepAction(.1), lift.liftBasket(), new SleepAction(.2), sliderTilt.basket(), turret.place()
        );

        Action TakeFromGround2 = new SequentialAction(
                linkage.take(), claw.open(), clawTilt.beforeTake(), clawRotate.horizontal()
        );
        Action PlaceInSlider2 = new SequentialAction(
                clawTilt.take(), new SleepAction(.2), claw.close(), new SleepAction(.2), clawRotate.inverted(), clawTilt.place(),new SleepAction(.2), linkage.place(), new SleepAction(.2),sliderClaw.open(), turret.take(),new SleepAction(.1), sliderTilt.take(),new SleepAction(.2), lift.takeFromLinkage(),new SleepAction(.2), claw.open(), sliderClaw.close()
        );

        Action BeforeTakeFromLinkage2 = new SequentialAction(
                lift.beforeLinkage(), new SleepAction(.05), sliderTilt.beforeLinkage()
        );
        Action PlaceOnBasket2 = new SequentialAction(
                sliderClaw.close(), new SleepAction(.2), lift.liftBasket(), new SleepAction(.3), sliderTilt.basket() ,turret.Basket()
        );

        Action TakeFromGround3 = new SequentialAction(
                linkage.take(), claw.open(), clawTilt.beforeTake(), pivot.take(), clawRotate.horizontal()
        );
        Action PlaceInSlider3 = new SequentialAction(
                clawTilt.take(), new SleepAction(.2), claw.close(), new SleepAction(.2), clawRotate.inverted(), pivot.init(), clawTilt.place(),new SleepAction(.2), linkage.place(), new SleepAction(.2),sliderClaw.open(), turret.take(),new SleepAction(.1), sliderTilt.take(),new SleepAction(.2), lift.takeFromLinkage(),new SleepAction(.2), claw.open(), sliderClaw.close()
        );

        Action BeforeTakeFromLinkage3 = new SequentialAction(
                lift.beforeLinkage(), new SleepAction(.05), sliderTilt.beforeLinkage()
        );
        Action PlaceOnBasket3 = new SequentialAction(
                sliderClaw.close(), new SleepAction(.2), lift.liftBasket(), new SleepAction(.3), sliderTilt.basket() ,turret.place()
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
                new SleepAction(.1),
                preload,
                new SleepAction(.2),
                sliderClaw.open(),
                new SleepAction(.1),
                g1,
                BeforeTakeFromLinkage,
                TakeFromGround,
                new SleepAction(.3),
                PlaceInSlider,
                new SleepAction(.4),
                PlaceOnBasket,
                new SleepAction(.3),
                place1,
                sliderClaw.open(),
                TakeFromGround2,
                g2,
                BeforeTakeFromLinkage2,
                new SleepAction(.4),
                PlaceInSlider2,
                new SleepAction(.4),
                PlaceOnBasket2,
                new SleepAction(.2),
                place2,
                new SleepAction(.3),
                sliderClaw.open(),
                new SleepAction(.2),
                TakeFromGround3,
                g3,
                BeforeTakeFromLinkage3,
                new SleepAction(.4),
                PlaceInSlider3,
                new SleepAction(.4),
                PlaceOnBasket3,
                new SleepAction(.3),
                place3,
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


        while (opModeIsActive() && !isStopRequested()) {

            Actions.runBlocking(
                    new ParallelAction(
                            autoSequence, pid
                    )
            );
        }
    }
}
