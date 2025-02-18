package org.firstinspires.ftc.teamcode.actions;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.systems.claw_controller;

import java.util.concurrent.TimeUnit;

public class Collect {
    Timing.Timer timer;

    Servo tilt, linkage, claw_rotate, claw, claw_pivot;
    DcMotorEx leftFront, leftBack, rightFront, rightBack;
    Gamepad gamepad1;
    double lim;
    claw_controller clawController;
    InTimer inTimer;


    public Collect(Servo claw, Servo tilt, Servo linkage, Servo claw_rotate, Servo claw_pivot, DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, double lim, Gamepad gamepad1){
        this.tilt = tilt;
        this.linkage = linkage;
        this.claw_rotate = claw_rotate;
        this.claw = claw;
        this.claw_pivot = claw_pivot;
        this.clawController = new claw_controller(this.claw);

        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
        this.lim = lim;
        this.gamepad1 = gamepad1;
        this.inTimer = new InTimer(leftFront, leftBack, rightFront, rightBack, 1, gamepad1);
    }

    public void takePos(){
        linkage.setPosition(Constants.LINKAGE_TAKE_POS);

        claw.setPosition(Constants.OPEN_CLAW);
        Constants.currentClawPos = Constants.ClawPos.OPEN_CLAW;

        claw_pivot.setPosition(Constants.CLAW_ASSEMBLY_TAKE);
        tilt.setPosition(Constants.TILT_BEFORE_TAKE);

        timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){inTimer.whileInTimer();}timer.pause();

        Constants.currentLinkageActionPos = Constants.LinkageActionPos.TAKE;
    }

    public void takeForThrow(){
        linkage.setPosition(Constants.LINKAGE_TAKE_POS);

        claw.setPosition(Constants.OPEN_CLAW);
        Constants.currentClawPos = Constants.ClawPos.OPEN_CLAW;

        claw_pivot.setPosition(Constants.CLAW_ASSEMBLY_TAKE);
        tilt.setPosition(Constants.TILT_THROW);

        timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){
            inTimer.whileInTimer();}timer.pause();

        Constants.currentLinkageActionPos = Constants.LinkageActionPos.TAKE;
    }

    public void placeInSlider(){
        tilt.setPosition(Constants.TILT_TAKE);
        timer = new Timing.Timer(125, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){inTimer.whileInTimer();}timer.pause();

        if(Constants.currentClawPos == Constants.ClawPos.OPEN_CLAW){
            claw.setPosition(Constants.CLOSE_CLAW);
            Constants.currentClawPos = Constants.ClawPos.CLOSE_CLAW;
            timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){inTimer.whileInTimer();}timer.pause();
        }

        claw_rotate.setPosition(Constants.ROTATE_PLACE_IN_SLIDER);
        Constants.currentClawRotatePos = Constants.ClawRotatePos.HORIZONTAL;

        tilt.setPosition(Constants.TILT_PLACE_IN_SLIDER);
        claw_pivot.setPosition(Constants.CLAW_ASSEMBLY_PLACE_IN_SLIDER);

        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){inTimer.whileInTimer();}timer.pause();

        linkage.setPosition(Constants.LINKAGE_PLACE_IN_SLIDER);

        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){inTimer.whileInTimer();}timer.pause();

        Constants.currentLinkageActionPos = Constants.LinkageActionPos.PLACE_IN_SLIDER;
    }

    public void placeInObservation(){
        tilt.setPosition(Constants.TILT_TAKE);
        timer = new Timing.Timer(200, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){inTimer.whileInTimer();}timer.pause();

        if(Constants.currentClawPos == Constants.ClawPos.OPEN_CLAW){
            claw.setPosition(Constants.CLOSE_CLAW);
            Constants.currentClawPos = Constants.ClawPos.CLOSE_CLAW;
            timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){inTimer.whileInTimer();}timer.pause();
        }

        tilt.setPosition(Constants.TILT_THROW);
        claw_pivot.setPosition(Constants.CLAW_ASSEMBLY_PLACE_IN_SLIDER);
        linkage.setPosition(Constants.LINKAGE_PLACE_IN_SLIDER);
        Constants.currentLinkageActionPos = Constants.LinkageActionPos.INIT;
    }

    public void switchServoAction_TakeSlider(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.PLACE_IN_SLIDER || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start(); while (!timer.done()){inTimer.whileInTimer();}timer.pause();
            takePos();
        }else if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE){
            timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start(); while (!timer.done()){inTimer.whileInTimer();}timer.pause();
            placeInSlider();
        }
    }

    public void switch_TakeThrow(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.PLACE_IN_SLIDER || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){inTimer.whileInTimer();}timer.pause();
            takeForThrow();
        }else if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE){
            timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){inTimer.whileInTimer();}timer.pause();
            placeInObservation();
        }
    }
}