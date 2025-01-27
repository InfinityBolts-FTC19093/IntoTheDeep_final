package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.actions.robot_drive;
import org.firstinspires.ftc.teamcode.actions.score_action;
import org.firstinspires.ftc.teamcode.actions.servo_linkage_action;
import org.firstinspires.ftc.teamcode.actions.servo_slider_action;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.RobotMap;
import org.firstinspires.ftc.teamcode.systems.claw_controller;
import org.firstinspires.ftc.teamcode.systems.sliderClaw_controller;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

public class ManualTeleOp extends LinearOpMode {

    Timing.Timer timer;
    int SliderPos = 0;

    RobotMap robot = new RobotMap(hardwareMap);

    slider_controller sliderController = new slider_controller(robot.slider);
    claw_controller clawController = new claw_controller(robot.claw);
    sliderClaw_controller sliderClawController = new sliderClaw_controller(robot.slider_claw);

    servo_linkage_action servoLinkageAction = new servo_linkage_action(robot.claw_tilt, robot.linkage);
    score_action scoreAction = new score_action();
    servo_slider_action servoSliderAction = new servo_slider_action();

    robot_drive robotDrive = new robot_drive(robot.leftFront, robot.leftBack, robot.rightFront, robot.rightBack, 1, robot.imu);

    @Override
    public void runOpMode() throws InterruptedException {
        robotDrive.robotCentricDrive();

        if (gamepad1.left_trigger >= 0.005 || gamepad1.right_trigger >= 0.005) {
            if (gamepad1.left_trigger >= 0.01) {
                robot.slider.setPower(-gamepad1.left_trigger);
                SliderPos = robot.slider.getCurrentPosition();
                sliderController.setTargetPosition(SliderPos);
                if (sliderController.pos() <= 0) {
                    sliderController.setTargetPosition(Constants.SLIDER_DOWN);
                }
            }
            if (gamepad1.right_trigger >= 0.01) {
                robot.slider.setPower(gamepad1.right_trigger);
                SliderPos = robot.slider.getCurrentPosition();
                sliderController.setTargetPosition(SliderPos);
            }
        } else if (!gamepad2.left_bumper) {sliderController.update();}
    }
}
