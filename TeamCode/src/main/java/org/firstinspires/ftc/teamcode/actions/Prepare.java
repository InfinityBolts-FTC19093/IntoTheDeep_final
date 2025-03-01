package org.firstinspires.ftc.teamcode.actions;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

import java.util.concurrent.TimeUnit;

public class Prepare {
    Timing.Timer timer;

    Servo slider_claw, tilt, rotate,claw;
    DcMotorEx leftFront, leftBack, rightFront, rightBack, slider;
    static slider_controller sliderController;
    Gamepad gamepad1;
    double lim;
    static robot_drive drive;

    public Prepare(Servo slider_claw, Servo tilt, Servo rotate, DcMotorEx slider, Servo claw, DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, double lim, Gamepad gamepad1){
        this.slider_claw = slider_claw;
        this.tilt = tilt;
        this.rotate = rotate;
        this.slider = slider;
        this.claw = claw;

        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
        this.gamepad1 = gamepad1;
        this.lim = lim;
    }

    public static void setSliderController(slider_controller controller){
        sliderController = controller;
    }

    public static void setDrive(robot_drive drive1){
        drive = drive1;
    }

    public void whileInTimer(){
        drive.robotCentricDrive(leftFront, leftBack, rightFront, rightBack, lim, gamepad1);
    }

    public void takeFromLinkage(){
        if(Constants.currentClawPos == Constants.ClawPos.OPEN_CLAW){
            claw.setPosition(Constants.CLOSE_CLAW);
            Constants.currentClawPos = Constants.ClawPos.CLOSE_CLAW;
            timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();
        }

        if(Constants.currentSliderClawPos == Constants.SliderClawPos.CLOSE_CLAW){
            slider_claw.setPosition(Constants.OPEN_CLAW);
            Constants.currentSliderClawPos = Constants.SliderClawPos.OPEN_CLAW;
            timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();
        }

        rotate.setPosition(Constants.TURRET_TAKE_FROM_LINKAGE);
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();

        tilt.setPosition(Constants.SLIDER_TILT_TAKE_FROM_LINKAGE);
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();

        sliderController.setTargetPosition(Constants.SLIDER_TAKE_FORM_LINKAGE);
        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();sliderController.update();}timer.pause();

        slider_claw.setPosition(Constants.CLOSE_CLAW);
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();

        Constants.currentSliderActionPos = Constants.SliderActionPos.TAKE_FOR_LINKAGE;
    }

    public void beforeTakeFromLinkage(){
        sliderController.setTargetPosition(Constants.SLIDER_BEFORE_TAKE_FORM_LINKAGE);
        sliderController.update();

        tilt.setPosition(Constants.SLIDER_TILT_BEFORE_TAKE_FROM_LINKAGE);
        slider_claw.setPosition(Constants.OPEN_CLAW);
        Constants.currentSliderClawPos = Constants.SliderClawPos.OPEN_CLAW;

        Constants.currentSliderActionPos = Constants.SliderActionPos.BEFORE_TAKE_FROM_LINKAGE;
    }

    public void takeFromHuman(){
        if(Constants.currentSliderClawPos == Constants.SliderClawPos.CLOSE_CLAW){
            slider_claw.setPosition(Constants.OPEN_CLAW);
            Constants.currentSliderClawPos = Constants.SliderClawPos.OPEN_CLAW;
            timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();
        }

        rotate.setPosition(Constants.TURRET_TAKE_HUMAN);
        tilt.setPosition(Constants.SLIDER_TILT_BEFORE_TAKE_FORM_HUMAN);
        timer = new Timing.Timer(75, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();
        tilt.setPosition(Constants.SLIDER_TILT_TAKE_FORM_HUMAN);

        sliderController.setTargetPosition(Constants.SLIDER_DOWN);

    }

    public void placeOnHighChamber(){
        if(Constants.currentSliderClawPos == Constants.SliderClawPos.OPEN_CLAW){
            slider_claw.setPosition(Constants.CLOSE_CLAW);
            Constants.currentSliderClawPos = Constants.SliderClawPos.CLOSE_CLAW;
            timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();
        }

        tilt.setPosition(Constants.SLIDER_TILT_PLACE_ON_HIGH_CHAMBER);
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();

        rotate.setPosition(Constants.TURRET_PLACE);
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();

        sliderController.setTargetPosition(Constants.SLIDER_HIGH_CHAMBER);

        Constants.previousSliderActionPos = Constants.currentSliderActionPos;
        Constants.currentSliderActionPos = Constants.SliderActionPos.PLACE_ON_CHAMBER;
    }

    public void placeOnHighBusket(){
        claw.setPosition(Constants.OPEN_CLAW);
        Constants.currentClawPos = Constants.ClawPos.OPEN_CLAW;

        slider_claw.setPosition(Constants.CLOSE_CLAW);
        Constants.currentSliderClawPos = Constants.SliderClawPos.CLOSE_CLAW;
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();

        sliderController.setTargetPosition(Constants.SLIDER_HIGH_BUSKET);
        timer = new Timing.Timer(200, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();sliderController.update();}timer.pause();

        tilt.setPosition(Constants.SLIDER_TILT_PLACE_IN_HIGH_BUSKET);
        rotate.setPosition(Constants.TURRET_PLACE);

        Constants.currentSliderActionPos = Constants.SliderActionPos.PLACE_IN_BUSKET;
    }

    public void placeOnLowBusket(){
        claw.setPosition(Constants.OPEN_CLAW);
        Constants.currentClawPos = Constants.ClawPos.OPEN_CLAW;

        slider_claw.setPosition(Constants.CLOSE_CLAW);
        Constants.currentSliderClawPos = Constants.SliderClawPos.CLOSE_CLAW;
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();

        sliderController.setTargetPosition(Constants.SLIDER_LOW_BUSKET);
        timer = new Timing.Timer(200, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();sliderController.update();}timer.pause();

        tilt.setPosition(Constants.SLIDER_TILT_PLACE_IN_LOW_BUSKET);
        rotate.setPosition(Constants.TURRET_PLACE);

        Constants.currentSliderActionPos = Constants.SliderActionPos.PLACE_IN_BUSKET;
    }
}
