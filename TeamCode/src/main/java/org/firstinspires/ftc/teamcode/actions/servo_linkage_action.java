package org.firstinspires.ftc.teamcode.actions;

import static org.firstinspires.ftc.teamcode.constants.Constants.currentLinkageActionPos;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.systems.autoClaw_controller;

import java.util.concurrent.TimeUnit;

public class servo_linkage_action {
    private Servo tilt, linkage, claw_rotate, claw, turret;
    Timing.Timer timer;
    autoClaw_controller autoClawController = new autoClaw_controller();

    public servo_linkage_action(Servo claw, Servo tilt, Servo linkage, Servo claw_rotate, Servo turret){
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
        tilt.setPosition(Constants.TILT_TAKE);

        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        autoClawController.take();

        timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        currentLinkageActionPos = Constants.LinkageActionPos.TAKE;
    }

    public void placeInSlider(){
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

        currentLinkageActionPos = Constants.LinkageActionPos.PLACE_IN_SLIDER;
    }

    public void placeInObservation(){
        if(Constants.currentClawPos == Constants.ClawPos.OPEN_CLAW){
            claw.setPosition(Constants.CLOSE_CLAW);
            timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        }

        tilt.setPosition(Constants.TILT_THROW);
        turret.setPosition(Constants.CLAW_ASSEMBLY_PLACE_IN_SLIDER);
        currentLinkageActionPos = Constants.LinkageActionPos.OSERVATION;
    }

    public void switch_TakeObservation(){
        if(currentLinkageActionPos == Constants.LinkageActionPos.PLACE_IN_SLIDER || currentLinkageActionPos == Constants.LinkageActionPos.INIT || currentLinkageActionPos == Constants.LinkageActionPos.OSERVATION){
            takePos();
        }else if(currentLinkageActionPos == Constants.LinkageActionPos.TAKE){
            placeInObservation();
        }

    }

    public void switchServoAction_TakeSlider(){
        if(currentLinkageActionPos == Constants.LinkageActionPos.PLACE_IN_SLIDER || currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            takePos();
        }else if(currentLinkageActionPos == Constants.LinkageActionPos.TAKE){
            placeInSlider();
        }
    }
}
