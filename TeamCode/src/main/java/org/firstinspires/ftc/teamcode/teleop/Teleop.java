package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.actions.G2_Action;
import org.firstinspires.ftc.teamcode.actions.robot_drive;
import org.firstinspires.ftc.teamcode.actions.score_action;
import org.firstinspires.ftc.teamcode.actions.servo_linkage_action;
import org.firstinspires.ftc.teamcode.actions.servo_slider_action;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.RobotMap;
import org.firstinspires.ftc.teamcode.systems.claw_controller;
import org.firstinspires.ftc.teamcode.systems.sliderClaw_controller;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

public class Teleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotMap robot = new RobotMap(hardwareMap);

        slider_controller sliderController = new slider_controller(robot.slider);
        claw_controller clawController = new claw_controller(robot.claw);
        sliderClaw_controller sliderClawController = new sliderClaw_controller(robot.slider_claw);

        servo_linkage_action servoLinkageAction = new servo_linkage_action(robot.claw_tilt, robot.linkage);

        score_action scoreAction = new score_action();
        servo_slider_action servoSliderAction = new servo_slider_action();
        G2_Action g2Action = new G2_Action();

        robot_drive robotDrive = new robot_drive(robot.leftFront, robot.leftBack, robot.rightFront, robot.rightBack, 1, robot.imu);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            robotDrive.robotCentricDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if (gamepad1.dpad_right) {clawController.open_close();}

            if (gamepad1.dpad_left) {sliderClawController.open_close();}

            if (gamepad1.dpad_down) {sliderController.setTargetPosition(Constants.SLIDER_DOWN);}

            if (gamepad1.left_bumper) {servoLinkageAction.switchServoAction();}

            if (gamepad1.y) {scoreAction.placeOnHighChamber();}

            if (gamepad1.a) {scoreAction.placeOnLowChamber();}

            if (gamepad1.x) {scoreAction.placeInHighBusket();}

            if (gamepad1.b) {scoreAction.placeInLowBusket();}

            if (gamepad2.right_stick_button && gamepad2.left_stick_button){g2Action.lev2Asent();}

            if(gamepad2.ps){g2Action.zeroPos();}

            sliderClawController.update();
            sliderController.update();
            clawController.update();
        }
    }
}