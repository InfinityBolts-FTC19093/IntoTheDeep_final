package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.RobotMap;
import org.firstinspires.ftc.teamcode.systems.linkage.angle_controller;
import org.firstinspires.ftc.teamcode.systems.linkage.claw_controller;
import org.firstinspires.ftc.teamcode.systems.linkage.rotateClaw_controller;
import org.firstinspires.ftc.teamcode.systems.linkage.tilt_controller;

@TeleOp (name = "TELEEOPP")
public class Teleop extends LinearOpMode {
    double  PrecisionDenominator=1, PrecisionDenominator2=1;

    public void robotCentricDrive(DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, double  lim) {
        double y =  gamepad1.left_stick_y;
        double x =  gamepad1.left_stick_x* 1;
        double rx = gamepad1.right_stick_x*1;

        rx/=PrecisionDenominator2;
        x/=PrecisionDenominator;
        y/=PrecisionDenominator;

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
//        sliderClaw_controller sliderClawController = new sliderClaw_controller(robot.slider_claw);
//        slider_controller sliderController = new slider_controller(robot.slider);

        claw_controller clawController = new claw_controller(robot.claw);
        angle_controller angleController = new angle_controller(robot.angle);
        rotateClaw_controller rotateController = new rotateClaw_controller(robot.claw_rotate);
        tilt_controller tiltController = new tilt_controller(robot.claw_tilt);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            robotCentricDrive(robot.leftFront, robot.leftBack, robot.rightFront, robot.rightBack, 1);

            if (gamepad1.dpad_right) {
                clawController.open_close();
            }

            if (gamepad1.a) {
                rotateController.rotate();
            }

            if (gamepad1.b) {
                rotateController.midRotate();
            }

            if (gamepad1.left_bumper) {
                tiltController.tiltTake();
            }

            if (gamepad1.right_bumper) {
                tiltController.tiltGive();
            }

            if (gamepad1.dpad_down) {
                angleController.changeAngle();
            }

            if (gamepad1.dpad_left) {
                angleController.giveAngle();
            }








//            if (gamepad1.dpad_left) {sliderClawController.open_close();}
//
//            if (gamepad1.dpad_down) {sliderController.setTargetPosition(Constants.SLIDER_DOWN);}
//
//            if (gamepad1.left_bumper) {servoLinkageAction.switchServoAction();}
//
//            if (gamepad1.y) {scoreAction.placeOnHighChamber();}
//
//            if (gamepad1.a) {scoreAction.placeOnLowChamber();}
//
//            if (gamepad1.x) {scoreAction.placeInHighBusket();}
//
//            if (gamepad1.b) {scoreAction.placeInLowBusket();}
//
//            if (gamepad2.right_bumper){ridicaAction.start();}

            clawController.update();
            rotateController.update();
            angleController.update();
            tiltController.update();


//            sliderClawController.update();
//            sliderController.update();
        }
    }
}