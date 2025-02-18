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
import org.firstinspires.ftc.teamcode.actions.Collect;
import org.firstinspires.ftc.teamcode.actions.Prepare;
import org.firstinspires.ftc.teamcode.actions.Score;
import org.firstinspires.ftc.teamcode.constants.RobotMap;

@Autonomous (name = "Human", group = "#")
public class Human extends LinearOpMode {
    int sliderPos;
    @Override
    public void runOpMode() {
        RobotMap robot = new RobotMap(hardwareMap);
        Pose2d startPose = new Pose2d(9, -61.5, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Prepare SliderAction        = new Prepare(robot.slider_claw, robot.slider_claw_tilt, robot.turret, robot.slider, robot.claw, robot.leftFront, robot.leftBack, robot.rightFront, robot.rightBack, 1, robot.gamepad1, sliderPos);
        Collect LinkageAction       = new Collect(robot.claw ,robot.claw_tilt, robot.linkage, robot.claw_rotate, robot.claw_pivot, robot.leftFront, robot.leftBack, robot.rightFront, robot.rightBack, 1, robot.gamepad1);
        Score ScoreAction           = new Score(robot.claw, robot.claw_tilt, robot.linkage, robot.claw_rotate, robot.claw_pivot, robot.slider_claw, robot.slider_claw_tilt, robot.turret, robot.slider, LinkageAction, SliderAction, robot.leftFront, robot.leftBack, robot.rightFront, robot.rightBack, 1, robot.gamepad1);

        actions.Update updateAuto   = new actions.Update(hardwareMap);
        actions.scoreAuto scoreAuto = new actions.scoreAuto(SliderAction, LinkageAction, ScoreAction);

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
            scoreAuto.Chamber(),
            preloadAction,
            new SleepAction(1),
            scoreAuto.Place(),
            humanAction,
            GTS1Action,
            scoreAuto.TakeFromHuman(),
            new SleepAction(.3),
            scoreAuto.Chamber(),
            chamber1,
            new SleepAction(.3),
            scoreAuto.Place(),
            new SleepAction(.3),
            GTS2Action,
            scoreAuto.TakeFromHuman(),
            new SleepAction(.3),
            scoreAuto.Chamber(),
            chamber2,
            new SleepAction(.3),
            scoreAuto.Place(),
            GTS3Action,
            scoreAuto.TakeFromHuman()

        );

        Action pid = new ParallelAction(
                updateAuto.updateAll()
        );



        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        autoSequence, pid
                )
        );
    }
}
