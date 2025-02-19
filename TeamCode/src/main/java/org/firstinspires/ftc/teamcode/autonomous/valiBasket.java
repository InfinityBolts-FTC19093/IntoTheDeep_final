package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
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
import org.firstinspires.ftc.teamcode.constants.RobotMap;
import org.firstinspires.ftc.teamcode.systems.sliderClaw_controller;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

@Autonomous(name = "VALI", group = "#")
public class valiBasket extends LinearOpMode {
    CollectAuto LinkageAction;
    PrepareAuto SliderAction;
    ScoreAuto ScoreAction;
    RobotMap robot;
    slider_controller sliderController;
    sliderClaw_controller sliderClawController;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotMap(hardwareMap);
        Pose2d startPose = new Pose2d(-33, -61.5, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        sliderController = new slider_controller(robot.slider);
        sliderClawController = new sliderClaw_controller(robot.slider_claw);

        SliderAction     = new PrepareAuto(robot.slider_claw, robot.slider_claw_tilt, robot.turret, robot.slider, robot.claw);
        LinkageAction    = new CollectAuto(robot.claw ,robot.claw_tilt, robot.linkage, robot.claw_rotate, robot.claw_pivot);
        ScoreAction      = new ScoreAuto(robot.claw, robot.claw_tilt, robot.linkage, robot.claw_rotate, robot.claw_pivot, robot.slider_claw, robot.slider_claw_tilt, robot.turret);

        actions.Update updateAuto   = new actions.Update(hardwareMap);
        actions.scoreAuto scoreAuto = new actions.scoreAuto(SliderAction, LinkageAction, ScoreAction, robot.slider);

        PrepareAuto.setSliderController(sliderController);
        ScoreAuto.setSliderController(sliderController);
        ScoreAuto.setsliderClawController(sliderClawController);
        ScoreAuto.setLinkageAction(LinkageAction);
        ScoreAuto.setSliderAction(SliderAction);

        TrajectoryActionBuilder safePose = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(-33.1, -61.5));

        TrajectoryActionBuilder PRELOAD = safePose.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(225));

        TrajectoryActionBuilder G1 = PRELOAD.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-48, -44), Math.toRadians(90));

        TrajectoryActionBuilder PLACE1 = G1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45));

        TrajectoryActionBuilder G2 = PLACE1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-58, -44), Math.toRadians(90));

        TrajectoryActionBuilder PLACE2 = G1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45));

        TrajectoryActionBuilder G3 = PLACE2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-55, -44), Math.toRadians(125));

        TrajectoryActionBuilder PLACE3 = G3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45));

        TrajectoryActionBuilder PARK = PLACE3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-35, -10), Math.toRadians(0))
                .strafeTo(new Vector2d(-20, -10));


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
                scoreAuto.BasketPreload(),
                //scoreAuto.Place(),
                preload,
                //scoreAuto.Place(),
                new SleepAction(1),
                g1,
                scoreAuto.TakeFromGround(),
                new SleepAction(1),
                scoreAuto.Basket(),
                place1,
                scoreAuto.Place(),
                new SleepAction(1)
//                g2,
//                scoreAuto.TakeFromGround(),
//                new SleepAction(1),
//                scoreAuto.Basket(),
//                place2,
//                scoreAuto.Place(),
//                new SleepAction(1),
//                g3,
//                scoreAuto.TakeFromGround(),
//                new SleepAction(1),
//                scoreAuto.Basket(),
//                place3,
//                scoreAuto.Place(),
//                new SleepAction(1),
//                park
        );

        Action pid = new ParallelAction(
                updateAuto.updateAll()
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
