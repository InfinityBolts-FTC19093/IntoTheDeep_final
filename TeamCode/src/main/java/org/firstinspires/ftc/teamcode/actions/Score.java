package org.firstinspires.ftc.teamcode.actions;

import static org.firstinspires.ftc.teamcode.constants.Constants.WAIT_FOR_LINKAGE_ACTION;
import static org.firstinspires.ftc.teamcode.constants.Constants.WAIT_FOR_SLIDER_ACTION;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.systems.sliderClaw_controller;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

import java.util.concurrent.TimeUnit;

public class Score {

     Timing.Timer timer;
     Servo claw, claw_tilt, linkage, claw_rotate, claw_pivot, slider_claw, slider_claw_tilt, turret;
     DcMotorEx slider;
     DcMotorEx leftFront, leftBack, rightFront, rightBack;
     Gamepad gamepad1;
     double lim;
     Collect LinkageAction;
     Prepare SliderAction;
     static slider_controller sliderController;
     sliderClaw_controller sliderClawController;
     InTimer inTimer;

    public Score(Servo claw, Servo claw_tilt, Servo linkage, Servo claw_rotate, Servo claw_pivot, Servo slider_claw, Servo slider_claw_tilt, Servo turret, DcMotorEx slider, Collect LinkageAction, Prepare SliderAction, DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, double lim, Gamepad gamepad1) {
        this.claw = claw;
        this.claw_tilt = claw_tilt;
        this.linkage = linkage;
        this.claw_rotate = claw_rotate;
        this.claw_pivot = claw_pivot;
        this.slider_claw = slider_claw;
        this.slider_claw_tilt = slider_claw_tilt;
        this.turret = turret;
        this.slider = slider;
        this.LinkageAction = LinkageAction;
        this.SliderAction = SliderAction;
        this.sliderClawController = new sliderClaw_controller(this.slider_claw);

        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
        this.lim = lim;
        this.gamepad1 = gamepad1;
        this.inTimer = new InTimer(leftFront, leftBack, rightFront, rightBack, 1, gamepad1);

    }


    public void whileInTimer(){
        inTimer.whileInTimer();
    }

    public static void setSliderController(slider_controller controller){
        sliderController = controller;
    }

    /** CHAMBER */

    public void placeOnHighChamber(){
        if(Constants.currentSliderActionPos == Constants.SliderActionPos.BEFORE_TAKE_FROM_LINKAGE){
            SliderAction.takeFromLinkage();
            timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();
        }

        SliderAction.placeOnHighChamber();

        Constants.currentScorePos = Constants.ScorePos.CHAMBER;
    }

    public void BasketPreload () {
        SliderAction.placeOnHighBusketLinkage();
    }

    public void placeOnLowChamber(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            LinkageAction.placeInSlider();
            timer = new Timing.Timer(WAIT_FOR_LINKAGE_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();
        }

        sliderController.setTargetPosition(Constants.SLIDER_LOW_CHAMBER);
        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();

        sliderController.setTargetPosition(Constants.SLIDER_LOW_CHAMBER-200);
        sliderClawController.setPos(Constants.OPEN_CLAW);

        Constants.currentScorePos = Constants.ScorePos.CHAMBER;
    }


    /** BUSKET */

    public void placeInHighBasket(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            LinkageAction.placeInSlider();
            timer = new Timing.Timer(WAIT_FOR_LINKAGE_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();
        }

        if(Constants.currentSliderActionPos == Constants.SliderActionPos.BEFORE_TAKE_FROM_LINKAGE){
            SliderAction.takeFromLinkage();
            timer = new Timing.Timer(WAIT_FOR_SLIDER_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();
        }

        SliderAction.placeOnHighBusket();

        Constants.currentScorePos = Constants.ScorePos.BUSKET;
    }



    public void placeInLowBusket(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            LinkageAction.placeInSlider();
            timer = new Timing.Timer(WAIT_FOR_LINKAGE_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();
        }

        if(Constants.currentSliderActionPos == Constants.SliderActionPos.PLACE_ON_CHAMBER || Constants.currentSliderActionPos == Constants.SliderActionPos.INIT){
            SliderAction.takeFromLinkage();
            timer = new Timing.Timer(WAIT_FOR_SLIDER_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();
        }

        SliderAction.placeOnLowBusket();

        Constants.currentScorePos = Constants.ScorePos.BUSKET;
    }

    public void take () {
        LinkageAction.switchServoAction_TakeSlider();
    }

    public void placeInBasket(){
        if(Constants.currentBasketPos == Constants.BasketPos.HIGH_BASKET){
            placeInHighBasket();
        }else{
            placeInLowBusket();
        }
    }

    public void observation(){
        LinkageAction.placeInObservation();
    }

    public void LinkagePlaceInSlider(){LinkageAction.placeInSlider2();}

    public void score(){
        if(Constants.currentScorePos == Constants.ScorePos.CHAMBER){
            sliderController.setTargetPosition(Constants.SLIDER_HIGH_CHAMBER+500);
            timer = new Timing.Timer(350, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();sliderController.update();}timer.pause();

            sliderClawController.setPos(Constants.OPEN_CLAW);
            Constants.currentSliderClawPos = Constants.SliderClawPos.OPEN_CLAW;

            SliderAction.takeFromHuman();
        }

        if(Constants.currentScorePos == Constants.ScorePos.BUSKET){
            if(Constants.currentSliderClawPos == Constants.SliderClawPos.CLOSE_CLAW){
                slider_claw.setPosition(Constants.OPEN_CLAW);
                Constants.currentSliderClawPos = Constants.SliderClawPos.OPEN_CLAW;
                timer = new Timing.Timer(350, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();
            }

            slider_claw_tilt.setPosition(Constants.SLIDER_TILT_BEFORE_TAKE_FROM_LINKAGE);
            timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();


            sliderController.setTargetPosition(Constants.SLIDER_TAKE_FORM_LINKAGE);
        }
    }

    public void collect(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE || Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE2){
            LinkageAction.take2();
        }else if (Constants.currentLinkageActionPos == Constants.LinkageActionPos.PLACE_IN_SLIDER){
            SliderAction.beforeTakeFromLinkage();
        }
    }
}
