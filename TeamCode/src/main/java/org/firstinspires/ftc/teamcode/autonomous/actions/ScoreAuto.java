package org.firstinspires.ftc.teamcode.autonomous.actions;

import static org.firstinspires.ftc.teamcode.constants.Constants.WAIT_FOR_LINKAGE_ACTION;
import static org.firstinspires.ftc.teamcode.constants.Constants.WAIT_FOR_SLIDER_ACTION;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.systems.sliderClaw_controller;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

import java.util.concurrent.TimeUnit;

public class ScoreAuto {

    Timing.Timer timer;
    Servo claw, claw_tilt, linkage, claw_rotate, claw_pivot, slider_claw, slider_claw_tilt, turret;
    DcMotorEx slider;
    CollectAuto LinkageAction;
    PrepareAuto SliderAction;
    static slider_controller sliderController;
    sliderClaw_controller sliderClawController;


    public ScoreAuto(Servo claw, Servo claw_tilt, Servo linkage, Servo claw_rotate, Servo claw_pivot, Servo slider_claw, Servo slider_claw_tilt, Servo turret, DcMotorEx slider, CollectAuto LinkageAction, PrepareAuto SliderAction) {
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
    }

    public static void setSliderController(slider_controller controller){
        sliderController = controller;
    }

    /** CHAMBER */

    public void placeOnHighChamber(){
        if(Constants.currentSliderActionPos == Constants.SliderActionPos.BEFORE_TAKE_FROM_LINKAGE){
            SliderAction.takeFromLinkage();
            timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();
        }

        SliderAction.placeOnHighChamber();

        Constants.currentScorePos = Constants.ScorePos.CHAMBER;
    }

    public void BasketPreload () {
        SliderAction.placeOnHighBusketLinkage();
    }

    /** BUSKET */

    public void placeInHighBasket(){
       LinkageAction.placeInSlider();
       timer = new Timing.Timer(WAIT_FOR_LINKAGE_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();

       SliderAction.takeFromLinkage();
       timer = new Timing.Timer(WAIT_FOR_SLIDER_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();

       SliderAction.placeOnHighBusket();

    }


    public void take () {
        LinkageAction.switchServoAction_TakeSlider();
    }

    public void score(){
        if(Constants.currentScorePos == Constants.ScorePos.CHAMBER){
            sliderController.setTargetPosition(Constants.SLIDER_HIGH_CHAMBER+500);
            timer = new Timing.Timer(350, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();

            sliderClawController.setPos(Constants.OPEN_CLAW);
            Constants.currentSliderClawPos = Constants.SliderClawPos.OPEN_CLAW;

            SliderAction.takeFromHuman();
        }

        if(Constants.currentScorePos == Constants.ScorePos.BUSKET){
            if(Constants.currentSliderClawPos == Constants.SliderClawPos.CLOSE_CLAW){
                slider_claw.setPosition(Constants.OPEN_CLAW);
                Constants.currentSliderClawPos = Constants.SliderClawPos.OPEN_CLAW;
                timer = new Timing.Timer(350, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();
            }

            slider_claw_tilt.setPosition(Constants.SLIDER_TILT_BEFORE_TAKE_FROM_LINKAGE);
            timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();


            sliderController.setTargetPosition(Constants.SLIDER_TAKE_FORM_LINKAGE);
        }
    }

}
