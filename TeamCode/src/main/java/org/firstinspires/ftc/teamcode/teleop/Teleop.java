package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.actions.G2_Action;
import org.firstinspires.ftc.teamcode.actions.score_action;
import org.firstinspires.ftc.teamcode.actions.servo_linkage_action;
import org.firstinspires.ftc.teamcode.actions.servo_slider_action;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.RobotMap;
import org.firstinspires.ftc.teamcode.systems.claw_controller;
import org.firstinspires.ftc.teamcode.systems.manualSlider_controller;
import org.firstinspires.ftc.teamcode.systems.sliderClaw_controller;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

public class Teleop extends LinearOpMode {

    public void robotCentricDrive(DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, double lim) {
        Constants.currentRobotDriveStatus = Constants.RobotDriveStatus.ROBOT_CENTRIC;
        double y = -gamepad1.left_stick_y;
        double x =  gamepad1.left_stick_x* 1;
        double rx = gamepad1.right_stick_x*1;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftPower = Clip(frontLeftPower,lim);
        backLeftPower = Clip(backLeftPower,lim);
        frontRightPower = Clip(frontRightPower,lim);
        backRightPower = Clip(backRightPower,lim);

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }

    double Clip(double Speed,double lim) {return Math.max(Math.min(Speed,lim),-lim);}

    @Override
    public void runOpMode() throws InterruptedException {
        RobotMap robot = new RobotMap(hardwareMap);

        slider_controller sliderController = new slider_controller(robot.slider);
        claw_controller clawController = new claw_controller(robot.claw);
        sliderClaw_controller sliderClawController = new sliderClaw_controller(robot.slider_claw);

        servo_linkage_action LinkageAction = new servo_linkage_action(robot.claw ,robot.claw_tilt, robot.linkage, robot.claw_rotate, robot.rotate_claw_assembly);
        score_action scoreAction = new score_action(robot.claw, robot.claw_tilt, robot.linkage, robot.claw_rotate, robot.rotate_claw_assembly, robot.slider_claw, robot.slider_claw_tilt, robot.slider_claw_rotate, robot.slider);
        servo_slider_action SliderAction = new servo_slider_action(robot.slider_claw, robot.slider_claw_tilt, robot.slider_claw_rotate, robot.slider);

        manualSlider_controller manualSliderController = new manualSlider_controller(robot.slider);
        G2_Action g2Action = new G2_Action(hardwareMap);


        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            robotCentricDrive(robot.leftFront, robot.leftBack, robot.rightFront, robot.rightBack, 1);

            if (gamepad1.dpad_right) {clawController.open_close();}

            if (gamepad1.dpad_left) {sliderClawController.open_close();}

            if (gamepad1.dpad_down) {sliderController.setTargetPosition(Constants.SLIDER_DOWN);}

            if (gamepad1.left_bumper) {LinkageAction.switchServoAction_TakeSlider();}

            if(gamepad1.right_bumper){LinkageAction.switch_TakeThrow();}

            if (gamepad1.a) {scoreAction.placeOnHighChamber();}

            if(gamepad1.b){scoreAction.placeInHighBusket();}

            if (gamepad1.x) {SliderAction.takeFromHuman();}

            if (gamepad2.right_stick_button && gamepad2.left_stick_button){g2Action.lev2Asent();}

            if(gamepad2.ps){g2Action.zeroPos();}

            manualSliderController.control(gamepad1.left_trigger, gamepad1.right_trigger);
            sliderClawController.update();
            sliderController.update();
            clawController.update();
        }
    }
}