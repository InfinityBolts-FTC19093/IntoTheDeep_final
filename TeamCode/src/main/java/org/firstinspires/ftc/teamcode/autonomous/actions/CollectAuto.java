package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.systems.claw_controller;
import org.firstinspires.ftc.teamcode.systems.linkage_controller;

import java.util.concurrent.TimeUnit;

public class CollectAuto {
    Timing.Timer timer;
    Servo slider_claw_tilt, linkage, claw_rotate, claw, claw_pivot;
    static linkage_controller linkageController;
    claw_controller clawController;
    ColorSensor rotateSensor, centerSensor;
    public CollectAuto(Servo claw, Servo slider_claw_tilt, Servo linkage, Servo claw_rotate, Servo claw_pivot, ColorSensor rotateSensor, ColorSensor centerSensor){
        this.slider_claw_tilt = slider_claw_tilt;
        this.linkage = linkage;
        this.claw_rotate = claw_rotate;
        this.claw = claw;
        this.claw_pivot = claw_pivot;
        this.rotateSensor = rotateSensor;
        this.centerSensor = centerSensor;
        this.clawController = new claw_controller(this.claw);
    }

    public static void setLinkageController(linkage_controller controller){
        linkageController = controller;
    }

    public void takePos(){
        linkage.setPosition(Constants.LINKAGE_TAKE_POS);

        claw.setPosition(Constants.OPEN_CLAW);
        Constants.currentClawPos = Constants.ClawPos.OPEN_CLAW;

        claw_pivot.setPosition(Constants.CLAW_ASSEMBLY_TAKE);
        slider_claw_tilt.setPosition(Constants.TILT_BEFORE_TAKE);

        timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();

        Constants.currentLinkageActionPos = Constants.LinkageActionPos.TAKE;
    }

    public void takeForThrow(){
        linkage.setPosition(Constants.LINKAGE_TAKE_POS);

        claw.setPosition(Constants.OPEN_CLAW);
        Constants.currentClawPos = Constants.ClawPos.OPEN_CLAW;

        claw_pivot.setPosition(Constants.CLAW_ASSEMBLY_TAKE);
        slider_claw_tilt.setPosition(Constants.TILT_THROW);

        timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();

        Constants.currentLinkageActionPos = Constants.LinkageActionPos.TAKE;
    }

    public void placeInSlider(){
        slider_claw_tilt.setPosition(Constants.TILT_TAKE);
        new SleepAction(.125);
        if(Constants.currentClawPos == Constants.ClawPos.OPEN_CLAW){
            claw.setPosition(Constants.CLOSE_CLAW);
            Constants.currentClawPos = Constants.ClawPos.CLOSE_CLAW;
            new SleepAction(.1);        }

        claw_rotate.setPosition(Constants.ROTATE_PLACE_IN_SLIDER);
        Constants.currentClawRotatePos = Constants.ClawRotatePos.HORIZONTAL;

        slider_claw_tilt.setPosition(Constants.TILT_PLACE_IN_SLIDER);
        claw_pivot.setPosition(Constants.CLAW_ASSEMBLY_PLACE_IN_SLIDER);

        new SleepAction(.1);
        linkage.setPosition(Constants.LINKAGE_PLACE_IN_SLIDER);

        new SleepAction(.05);
        Constants.currentLinkageActionPos = Constants.LinkageActionPos.PLACE_IN_SLIDER;
    }

    public void placeInObservation(){
        slider_claw_tilt.setPosition(Constants.TILT_TAKE);
        timer = new Timing.Timer(200, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();

        if(Constants.currentClawPos == Constants.ClawPos.OPEN_CLAW){
            claw.setPosition(Constants.CLOSE_CLAW);
            Constants.currentClawPos = Constants.ClawPos.CLOSE_CLAW;
            timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();
        }

        slider_claw_tilt.setPosition(Constants.TILT_THROW);
        claw_pivot.setPosition(Constants.CLAW_ASSEMBLY_PLACE_IN_SLIDER);
        linkage.setPosition(Constants.LINKAGE_PLACE_IN_SLIDER);
        Constants.currentLinkageActionPos = Constants.LinkageActionPos.INIT;
    }

    public void switchServoAction_TakeSlider(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.PLACE_IN_SLIDER || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start(); while (!timer.done())timer.pause();
            takePos();
        }else if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE){
            timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start(); while (!timer.done())timer.pause();
            placeInSlider();
        }
    }

    public void switch_TakeThrow(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.PLACE_IN_SLIDER || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();
            takeForThrow();
        }else if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE){
            timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();
            placeInObservation();
        }
    }
}