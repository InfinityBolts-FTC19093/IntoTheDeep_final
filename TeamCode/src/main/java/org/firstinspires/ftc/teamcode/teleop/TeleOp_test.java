package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.constants.Constants.WAIT_FOR_LINKAGE_ACTION;
import static org.firstinspires.ftc.teamcode.constants.Constants.WAIT_FOR_SLIDER_ACTION;


import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.actions.Collect;
import org.firstinspires.ftc.teamcode.actions.Prepare;
import org.firstinspires.ftc.teamcode.actions.Score;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.RobotMap;
import org.firstinspires.ftc.teamcode.systems.sliderClaw_controller;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

import java.util.concurrent.TimeUnit;

@Disabled
@TeleOp(name = "TeleOp-Test", group = "#")
public class TeleOp_test extends LinearOpMode {

    RobotMap robot;
    Timing.Timer timer;
    slider_controller sliderController;
    Collect LinkageAction;
    Prepare SliderAction;
    Score scoreAction;
    sliderClaw_controller sliderClawController;

    public void takePos(){
        robot.linkage.setPosition(Constants.LINKAGE_TAKE_POS);

        robot.claw.setPosition(Constants.OPEN_CLAW);
        robot.claw_pivot.setPosition(Constants.CLAW_ASSEMBLY_TAKE);
        robot.claw_tilt.setPosition(Constants.TILT_BEFORE_TAKE);

        //timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileTimerNotDone();}timer.pause();
        sleep(50);
        //autoClawController.take();
        robot.claw_rotate.setPosition(Constants.ROTATE_TAKE_HORIONTAL);

        timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileTimerNotDone();}timer.pause();

        Constants.currentLinkageActionPos = Constants.LinkageActionPos.TAKE;
    }

    public void placeInSlider(){
        robot.claw_tilt.setPosition(Constants.TILT_TAKE);
        //timer = new Timing.Timer(125, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileTimerNotDone();}timer.pause();
        sleep(125);

        if(Constants.currentClawPos == Constants.ClawPos.OPEN_CLAW){
            robot.claw.setPosition(Constants.CLOSE_CLAW);
            timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileTimerNotDone();}timer.pause();
        }

        robot.claw_rotate.setPosition(Constants.ROTATE_PLACE_IN_SLIDER);

        robot.claw_tilt.setPosition(Constants.TILT_PLACE_IN_SLIDER);
        robot.claw_pivot.setPosition(Constants.CLAW_ASSEMBLY_PLACE_IN_SLIDER);

        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileTimerNotDone();}timer.pause();

        robot.linkage.setPosition(Constants.LINKAGE_PLACE_IN_SLIDER);

        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileTimerNotDone();}timer.pause();

        Constants.currentLinkageActionPos = Constants.LinkageActionPos.PLACE_IN_SLIDER;
    }

    public void placeInObservation(){
        if(Constants.currentClawPos == Constants.ClawPos.OPEN_CLAW){
            robot.claw.setPosition(Constants.CLOSE_CLAW);
            timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileTimerNotDone();}timer.pause();
        }

        robot.claw_tilt.setPosition(Constants.TILT_THROW);
        robot.claw_pivot.setPosition(Constants.CLAW_ASSEMBLY_PLACE_IN_SLIDER);
        Constants.currentLinkageActionPos = Constants.LinkageActionPos.TAKE;
    }

    public void switchServoAction_TakeSlider(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.PLACE_IN_SLIDER || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start(); while (!timer.done()){whileTimerNotDone();}timer.pause();
            takePos();
        }else if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE){
            timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start(); while (!timer.done()){whileTimerNotDone();}timer.pause();
            placeInSlider();
        }
    }

    public void switch_TakeThrow(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.PLACE_IN_SLIDER || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileTimerNotDone();}timer.pause();
            takePos();
        }else if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE){
            timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileTimerNotDone();}timer.pause();
            placeInObservation();
        }
    }



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

    public void takeFromLinkage(){
        if(Constants.currentSliderClawPos == Constants.SliderClawPos.CLOSE_CLAW){
            robot.slider_claw.setPosition(Constants.OPEN_CLAW);
            timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileTimerNotDone();}timer.pause();
        }

        robot.turret.setPosition(Constants.TURRET_TAKE_FROM_LINKAGE);
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileTimerNotDone();}timer.pause();

        robot.slider_claw_tilt.setPosition(Constants.SLIDER_TILT_TAKE_FROM_LINKAGE);
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileTimerNotDone();}timer.pause();

        sliderController.setTargetPosition(Constants.SLIDER_TAKE_FORM_LINKAGE);
        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileTimerNotDone();}timer.pause();

        robot.slider_claw.setPosition(Constants.CLOSE_CLAW);
        Constants.currentSliderClawPos = Constants.SliderClawPos.CLOSE_CLAW;
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileTimerNotDone();}timer.pause();

        Constants.currentSliderActionPos = Constants.SliderActionPos.TAKE_FOR_LINKAGE;
    }

    public void placeOnHighChamber(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            LinkageAction.placeInSlider();
            timer = new Timing.Timer(WAIT_FOR_LINKAGE_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileTimerNotDone();}timer.pause();
        }

        if(Constants.currentSliderActionPos == Constants.SliderActionPos.PLACE_ON_CHAMBER || Constants.currentSliderActionPos == Constants.SliderActionPos.INIT){
            SliderAction.takeFromLinkage();
            timer = new Timing.Timer(WAIT_FOR_SLIDER_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileTimerNotDone();}timer.pause();
        }

        SliderAction.placeOnHighChamber();
        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileTimerNotDone();}timer.pause();

        Constants.currentScorePos = Constants.ScorePos.CHAMBER;
    }

    public void placeOnHighBusket(){
        robot.claw.setPosition(Constants.OPEN_CLAW);
        sliderController.setTargetPosition(Constants.SLIDER_HIGH_BUSKET);
        timer = new Timing.Timer(200, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileTimerNotDone();}timer.pause();

        robot.slider_claw_tilt.setPosition(Constants.SLIDER_TILT_PLACE_IN_HIGH_BUSKET);
        robot.turret.setPosition(Constants.TURRET_PLACE);

        Constants.currentSliderActionPos = Constants.SliderActionPos.PLACE_IN_BUSKET;
    }

    public void RotateClaw(){
        //timer = new Timing.Timer(100, TimeUnit.MILLISECONDS); while (!timer.done()){whileTimerNotDone();} timer.pause();
        sleep(100);
        if(Constants.currentClawRotatePos == Constants.ClawRotatePos.HORIZONTAL){
            robot.claw_rotate.setPosition(Constants.ROTATE_TAKE_VERTICAL);
            //timer = new Timing.Timer(50, TimeUnit.MILLISECONDS); while (!timer.done()){whileTimerNotDone();} timer.pause();
            sleep(50);
            Constants.currentClawRotatePos = Constants.ClawRotatePos.VERTICAL;
        } else{
            robot.claw_rotate.setPosition(Constants.ROTATE_TAKE_HORIONTAL);
            //timer = new Timing.Timer(50, TimeUnit.MILLISECONDS); while (!timer.done()){whileTimerNotDone();} timer.pause();
            sleep(50);
            Constants.currentClawRotatePos = Constants.ClawRotatePos.HORIZONTAL;

        }
    }

    public void whileTimerNotDone(){
        robotCentricDrive(robot.leftFront, robot.leftBack, robot.rightFront, robot.rightBack, 1);
        sliderController.update();
    }

    @Override
    public void runOpMode() {
        robot = new RobotMap(hardwareMap);
        sliderController = new slider_controller(robot.slider);
        sliderClawController = new sliderClaw_controller(robot.slider_claw);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            robotCentricDrive(robot.leftFront, robot.leftBack, robot.rightFront, robot.rightBack, 1);

            if(gamepad1.left_bumper){
                switchServoAction_TakeSlider();
            }
            if(gamepad1.right_bumper){
                switch_TakeThrow();
            }
            if(gamepad1.dpad_down){
                takeFromLinkage();
            }
            if(gamepad1.dpad_up){
                placeOnHighBusket();
            }

            if(gamepad1.a){
                RotateClaw();
            }

            if(gamepad1.b){
                sliderClawController.open_close();
            }

            sliderController.update();
            sliderClawController.update();

            telemetry.addData("slider", robot.slider.getCurrentPosition());
            telemetry.update();
        }
    }
}
