package org.firstinspires.ftc.teamcode.actions;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.systems.autoClaw_controller;

import java.util.concurrent.TimeUnit;

public class Collect {
    Servo tilt, linkage, claw_rotate, claw, turret;
    Timing.Timer timer;

    public Collect(Servo claw, Servo tilt, Servo linkage, Servo claw_rotate, Servo turret){
        this.tilt = tilt;
        this.linkage = linkage;
        this.claw_rotate = claw_rotate;
        this.claw = claw;
        this.turret = turret;
    }

    public void takePos(){
        linkage.setPosition(Constants.LINKAGE_TAKE_POS);

        claw.setPosition(Constants.OPEN_CLAW);
        turret.setPosition(Constants.CLAW_ASSEMBLY_TAKE);
        tilt.setPosition(Constants.TILT_BEFORE_TAKE);

        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        //autoClawController.take();

        timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        Constants.currentLinkageActionPos = Constants.LinkageActionPos.TAKE;
    }

    public void placeInSlider(){
        tilt.setPosition(Constants.TILT_TAKE);
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        if(Constants.currentClawPos == Constants.ClawPos.OPEN_CLAW){
            claw.setPosition(Constants.CLOSE_CLAW);
            timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        }

        claw_rotate.setPosition(Constants.ROTATE_PLACE_IN_SLIDER);

        tilt.setPosition(Constants.TILT_PLACE_IN_SLIDER);
        turret.setPosition(Constants.CLAW_ASSEMBLY_PLACE_IN_SLIDER);

        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        linkage.setPosition(Constants.LINKAGE_PLACE_IN_SLIDER);

        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        Constants.currentLinkageActionPos = Constants.LinkageActionPos.PLACE_IN_SLIDER;
    }

    public void placeInObservation(){
        if(Constants.currentClawPos == Constants.ClawPos.OPEN_CLAW){
            claw.setPosition(Constants.CLOSE_CLAW);
            timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        }

        tilt.setPosition(Constants.TILT_THROW);
        turret.setPosition(Constants.CLAW_ASSEMBLY_PLACE_IN_SLIDER);
        Constants.currentLinkageActionPos = Constants.LinkageActionPos.INIT;
    }

    public void switch_TakeThrow(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.PLACE_IN_SLIDER || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            timer = new Timing.Timer(150, TimeUnit.MILLISECONDS); while (!timer.done()); timer.pause();
            takePos();
        }else if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE){
            timer = new Timing.Timer(150, TimeUnit.MILLISECONDS); while (!timer.done()); timer.pause();
            placeInObservation();
        }

    }

    public void switchServoAction_TakeSlider(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.PLACE_IN_SLIDER || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            timer = new Timing.Timer(150, TimeUnit.MILLISECONDS); while (!timer.done()); timer.pause();
            takePos();
        }else if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE){
            timer = new Timing.Timer(150, TimeUnit.MILLISECONDS); while (!timer.done()); timer.pause();
            placeInSlider();
        }
    }
}
