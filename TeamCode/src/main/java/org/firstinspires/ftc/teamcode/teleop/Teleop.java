package org.firstinspires.ftc.teamcode.teleop;
import static org.firstinspires.ftc.teamcode.constants.Constants.WAIT_FOR_LINKAGE_ACTION;
import static org.firstinspires.ftc.teamcode.constants.Constants.WAIT_FOR_SLIDER_ACTION;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.actions.G2_Action;
import org.firstinspires.ftc.teamcode.actions.Score;
import org.firstinspires.ftc.teamcode.actions.Collect;
import org.firstinspires.ftc.teamcode.actions.Prepare;
import org.firstinspires.ftc.teamcode.actions.robot_drive;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.RobotMap;
import org.firstinspires.ftc.teamcode.systems.claw_controller;
import org.firstinspires.ftc.teamcode.systems.manualSlider_controller;
import org.firstinspires.ftc.teamcode.systems.sliderClaw_controller;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "TeleOp", group = "#")
public class Teleop extends LinearOpMode {
    RobotMap robot;
    slider_controller sliderController;
    claw_controller clawController;
    sliderClaw_controller sliderClawController;
    manualSlider_controller manualSliderController;
    Collect LinkageAction;
    Prepare SliderAction;
    Score scoreAction;
    robot_drive drive;
    G2_Action g2Action;
    Timing.Timer timer;

    @Override
    public void runOpMode() {
        robot = new RobotMap(hardwareMap);

        sliderController         = new slider_controller(robot.slider);
        clawController           = new claw_controller(robot.claw);
        sliderClawController     = new sliderClaw_controller(robot.slider_claw);
        manualSliderController   = new manualSlider_controller(robot.slider);

        SliderAction     = new Prepare(robot.slider_claw, robot.slider_claw_tilt, robot.slider_claw_rotate, robot.slider);
        LinkageAction    = new Collect(robot.claw ,robot.claw_tilt, robot.linkage, robot.claw_rotate, robot.turret, timer);
        scoreAction      = new Score(robot.claw, robot.claw_tilt, robot.linkage, robot.claw_rotate, robot.turret, robot.slider_claw, robot.slider_claw_tilt, robot.slider_claw_rotate, robot.slider, LinkageAction, SliderAction);

        drive            = new robot_drive(robot.leftFront, robot.leftBack,robot.rightFront,robot.rightBack, 1,  robot.gamepad1);
        g2Action         = new G2_Action(robot.claw, robot.slider_claw, robot.claw_tilt, robot.claw_rotate, robot.turret, robot.linkage, robot.slider_claw_rotate, robot.slider, robot.unghi_robot);

        telemetry.addData("Data", "merge");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.robotCentricDrive(robot.leftFront, robot.leftBack, robot.rightFront, robot.rightBack, 1, gamepad1);

            if(gamepad1.dpad_right) {clawController.open_close();}

            if(gamepad1.dpad_left) {sliderClawController.open_close();}

            if(gamepad1.dpad_up) {scoreAction.rotate();}

            if(gamepad1.dpad_down) {sliderController.setTargetPosition(Constants.SLIDER_DOWN);}

            if(gamepad1.left_bumper) {scoreAction.throwS();}

            if(gamepad1.right_bumper){scoreAction.take();}

            if(gamepad1.a) {scoreAction.placeOnHighChamber();}

            if(gamepad1.b) {scoreAction.placeInHighBusket();}

            if(gamepad1.x) {SliderAction.takeFromHuman();}

            if(gamepad2.right_stick_button && gamepad2.left_stick_button){g2Action.lev2Asent();}

            if(gamepad2.ps){g2Action.zeroPos();}

            manualSliderController.control(gamepad1.left_trigger, gamepad1.right_trigger);
            sliderClawController.update();
            sliderController.update();
            clawController.update();
        }
    }
}