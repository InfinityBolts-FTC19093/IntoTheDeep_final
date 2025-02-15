package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.actions.G2_Action;
import org.firstinspires.ftc.teamcode.actions.robot_drive;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.RobotMap;
import org.firstinspires.ftc.teamcode.systems.manualSlider_controller;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "ManualTeleOp", group = "TEST")
public class ManualTeleOp extends LinearOpMode {

    public void robotCentricDrive(DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, double lim) {
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

    Timing.Timer timer;

    int SliderPos = 0;
    public double Linkage = 0, SliderClawRotate = 0.5, Claw = 0, SliderClaw = 0, SliderClawTilt = 0.5, ClawRotate=0, ClawTilt = 0, UnghiRobot = 0, clawAssembly = 0;
    public static int Wait = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        RobotMap robot = new RobotMap(hardwareMap);
        slider_controller sliderController = new slider_controller(robot.slider);
        manualSlider_controller manualSliderController = new manualSlider_controller(robot.slider);
        G2_Action g2Action = new G2_Action(robot.claw, robot.slider_claw, robot.claw_tilt, robot.claw_rotate, robot.turret, robot.linkage, robot.slider_claw_rotate, robot.slider, robot.unghi_robot);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            robotCentricDrive(robot.leftFront, robot.leftBack, robot.rightFront, robot.rightBack, 1);

            //Slider
//            if (gamepad1.left_trigger >= 0.005 || gamepad1.right_trigger >= 0.005) {
//                if (gamepad1.left_trigger >= 0.01) {
//                    robot.slider.setPower(-gamepad1.left_trigger);
//                    SliderPos = robot.slider.getCurrentPosition();
//                    sliderController.setTargetPosition(SliderPos);
//                    if (sliderController.pos() <= 0) {
//                        sliderController.setTargetPosition(Constants.SLIDER_DOWN);
//                    }
//                }
//                if (gamepad1.right_trigger >= 0.01) {
//                    robot.slider.setPower(gamepad1.right_trigger);
//                    SliderPos = robot.slider.getCurrentPosition();
//                    sliderController.setTargetPosition(SliderPos);
//                }
//            }


            //claw
            if(gamepad1.b){
                Claw += 0.05;
                timer = new Timing.Timer(Wait, TimeUnit.MILLISECONDS); while (!timer.done()); timer.pause();
            }

            if(gamepad1.x){
                Claw -= 0.05;
                timer = new Timing.Timer(Wait, TimeUnit.MILLISECONDS); while (!timer.done()); timer.pause();
            }


            //Slider Claw
            if(gamepad1.y){
                SliderClaw += 0.05;
                timer = new Timing.Timer(Wait, TimeUnit.MILLISECONDS); while (!timer.done()); timer.pause();
            }

            if(gamepad1.a){
                SliderClaw -= 0.05;
                timer = new Timing.Timer(Wait, TimeUnit.MILLISECONDS); while (!timer.done()); timer.pause();
            }


            //Linkage
            if(gamepad1.dpad_up){
                Linkage +=0.05;
                timer = new Timing.Timer(Wait, TimeUnit.MILLISECONDS); while (!timer.done()); timer.pause();
            }

            if(gamepad1.dpad_down){
                Linkage -=0.05;
                timer = new Timing.Timer(Wait, TimeUnit.MILLISECONDS); while (!timer.done()); timer.pause();
            }


            //Slider Claw Rotate
            if(gamepad1.dpad_right){
                SliderClawRotate += 0.05;
                timer = new Timing.Timer(Wait, TimeUnit.MILLISECONDS); while (!timer.done()); timer.pause();
            }

            if(gamepad1.dpad_left){
                SliderClawRotate -= 0.05;
                timer = new Timing.Timer(Wait, TimeUnit.MILLISECONDS); while (!timer.done()); timer.pause();
            }


            //Slider Claw Tilt
//            if(gamepad1.right_stick_y >0.2 || gamepad1.right_stick_y < -0.2 ){
//                SliderClawTilt = gamepad1.right_stick_y;
//                timer = new Timing.Timer(Wait, TimeUnit.MILLISECONDS); while (!timer.done()); timer.pause();
//            }

            //Claw Rotate
            if(gamepad1.left_bumper){
                ClawRotate -= 0.05;
                timer = new Timing.Timer(Wait, TimeUnit.MILLISECONDS); while (!timer.done()); timer.pause();
            }

            if(gamepad1.right_bumper){
                ClawRotate += 0.05;
                timer = new Timing.Timer(Wait, TimeUnit.MILLISECONDS); while (!timer.done()); timer.pause();
            }


            //Claw Tilt
            if(gamepad1.share){
                ClawTilt -=0.05;
                timer = new Timing.Timer(Wait, TimeUnit.MILLISECONDS); while (!timer.done()); timer.pause();
            }

            if(gamepad1.options){
                ClawTilt +=0.05;
                timer = new Timing.Timer(Wait, TimeUnit.MILLISECONDS); while (!timer.done()); timer.pause();
            }


            //Unghi Robot
            if(gamepad1.touchpad && gamepad1.touchpad_finger_1_x < -0.2){
                UnghiRobot -=0.05;
                timer = new Timing.Timer(Wait, TimeUnit.MILLISECONDS); while (!timer.done()); timer.pause();
            }

            if(gamepad1.touchpad && gamepad1.touchpad_finger_1_x > 0.2){
                UnghiRobot +=0.05;
                timer = new Timing.Timer(Wait, TimeUnit.MILLISECONDS); while (!timer.done()); timer.pause();
            }


            //Rotate Claw Assembly
            if(gamepad1.left_stick_button){
                clawAssembly -=0.05;
                timer = new Timing.Timer(Wait, TimeUnit.MILLISECONDS); while (!timer.done()); timer.pause();
            }

            if(gamepad1.right_stick_button){
                clawAssembly +=0.05;
                timer = new Timing.Timer(Wait, TimeUnit.MILLISECONDS); while (!timer.done()); timer.pause();
            }

            //Reset Pos
            if(gamepad1.ps){g2Action.zeroPos();}


            //linkage servos
            robot.claw.setPosition(Claw);
            robot.linkage.setPosition(Linkage);
            robot.claw_rotate.setPosition(ClawRotate);
            robot.claw_tilt.setPosition(ClawTilt);

            //slider Servos
            robot.slider_claw.setPosition(SliderClaw);
            robot.slider_claw_rotate.setPosition(SliderClawRotate);
            robot.slider_claw_tilt.setPosition(SliderClawTilt);

            //unghi robot
            robot.unghi_robot.setPosition(UnghiRobot);

            robot.turret.setPosition(clawAssembly);


            manualSliderController.control(gamepad1.left_trigger, gamepad1.right_trigger);

            sliderController.update();


            telemetry.addData("Claw-Pos", Claw);
            telemetry.addData("","");
            telemetry.addData("Slider-Claw-Pos", SliderClaw);
            telemetry.addData("","");
            telemetry.addData("Linkage-Pos", Linkage);
            telemetry.addData("","");
            telemetry.addData("Slider-Claw-Rotate-Pos", SliderClawRotate);
            telemetry.addData("Slider-Claw-Tilt-Pos", SliderClawTilt);
            telemetry.addData("Rotate-Claw-Assembly", clawAssembly);
            telemetry.addData("","");
            telemetry.addData("Claw-Rotate-Pos", ClawRotate);
            telemetry.addData("Claw-Tilt-Pos", ClawTilt);
            telemetry.addData("","");
            telemetry.addData("Unghi-Robot", UnghiRobot);
            telemetry.update();
        }
    }
}
