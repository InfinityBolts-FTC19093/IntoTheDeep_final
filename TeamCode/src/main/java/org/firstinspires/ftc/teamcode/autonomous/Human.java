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

@Autonomous (name = "Human", group = "#")
public class Human extends LinearOpMode {
    int sliderPos;
    @Override
    public void runOpMode() {
        RobotMap robot      = new RobotMap(hardwareMap);
        Pose2d startPose    = new Pose2d(9, -61.5, Math.toRadians(90));
        MecanumDrive drive  = new MecanumDrive(hardwareMap, startPose);

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

        Action PreLoad = new SequentialAction(
                lift.liftChamber(), sliderTilt.chamber()
        );

        Action Place= new SequentialAction(
                lift.liftPlace(), new SleepAction(.3), sliderClaw.open()
        );

        Action Human = new SequentialAction(
                sliderTilt.human(), sliderClaw.open(), lift.liftDown()
        );

        Action BeforeTakeFromGround = new SequentialAction(
                claw.open(), linkage.take(), clawTilt.beforeTake()
        );

        Action TakeFromGround = new SequentialAction(
                clawTilt.take(), new SleepAction(.2), claw.close(), new SleepAction(.1), clawTilt.place(), linkage.place()
        );

        Action ThrowHuman = new SequentialAction(
                linkage.take(), clawTilt.place(), new SleepAction(.1), claw.open(), new SleepAction(.05), linkage.place(), clawTilt.beforeTake()
        );

        Action TakeFromHuman = new SequentialAction(
                sliderClaw.close(), new SleepAction(.1), lift.liftChamber(), sliderTilt.chamber()
        );

        TrajectoryActionBuilder safePose = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(9, -61.4 ));

        TrajectoryActionBuilder PRELOAD = safePose.endTrajectory().fresh()
                .strafeTo(new Vector2d(8, -33), null, new ProfileAccelConstraint(-70, 70));

        TrajectoryActionBuilder HUMAN = PRELOAD.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(34, -34), Math.toRadians(45))
                .turn(Math.toRadians(-100))
                .turn(Math.toRadians(100))
                .strafeToLinearHeading(new Vector2d(40, -34), Math.toRadians(45))
                .turn(Math.toRadians(-100))
                .turn(Math.toRadians(100))
                .strafeToLinearHeading(new Vector2d(46, -34) ,Math.toRadians(45))
                .turn(Math.toRadians(-100));

        TrajectoryActionBuilder GTS1 = HUMAN.endTrajectory().fresh()
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


        Action autoSequence = new SequentialAction(
            safepose,
            preloadAction,
            new SleepAction(1),
            humanAction,
            GTS1Action,
            new SleepAction(.3),
            chamber1,
            new SleepAction(.3),
            new SleepAction(.3),
            GTS2Action,
            new SleepAction(.3),
            chamber2,
            new SleepAction(.3),
            GTS3Action
        );

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        autoSequence
                )
        );
    }
}
