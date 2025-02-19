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
    static Servo claw, claw_tilt, linkage, claw_rotate, claw_pivot, slider_claw, slider_claw_tilt, turret;
    static CollectAuto LinkageAction;
    static PrepareAuto SliderAction;
    static slider_controller sliderController;
    static sliderClaw_controller sliderClawController;


    public ScoreAuto(Servo claw1, Servo claw_tilt1, Servo linkage1, Servo claw_rotate1, Servo claw_pivot1, Servo slider_claw1, Servo slider_claw_tilt1, Servo turret1) {
        claw = claw1;
        claw_tilt = claw_tilt1;
        linkage = linkage1;
        claw_rotate = claw_rotate1;
        claw_pivot = claw_pivot1;
        slider_claw = slider_claw1;
        slider_claw_tilt = slider_claw_tilt1;
        turret = turret1;
    }

    public static void setSliderController(slider_controller controller){
        sliderController = controller;
    }

    public static void setsliderClawController(sliderClaw_controller controller){
        sliderClawController = controller;
    }

    public static void setLinkageAction(CollectAuto collectAuto){
        LinkageAction = collectAuto;
    }

    public static void setSliderAction(PrepareAuto prepareAuto){
        SliderAction = prepareAuto;
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

    public void placeOnLowChamber(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            LinkageAction.placeInSlider();
            timer = new Timing.Timer(WAIT_FOR_LINKAGE_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();
        }

        sliderController.setTargetPosition(Constants.SLIDER_LOW_CHAMBER);
        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();

        sliderController.setTargetPosition(Constants.SLIDER_LOW_CHAMBER-200);
        sliderClawController.setPos(Constants.OPEN_CLAW);

        Constants.currentScorePos = Constants.ScorePos.CHAMBER;
    }


    /** BUSKET */

    public void placeInHighBasket(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            LinkageAction.placeInSlider();
            timer = new Timing.Timer(WAIT_FOR_LINKAGE_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();
        }

        if(Constants.currentSliderActionPos == Constants.SliderActionPos.BEFORE_TAKE_FROM_LINKAGE){
            SliderAction.takeFromLinkage();
            timer = new Timing.Timer(WAIT_FOR_SLIDER_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();
        }

        SliderAction.placeOnHighBusket();

        Constants.currentScorePos = Constants.ScorePos.BUSKET;
    }



    public void placeInLowBusket(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            LinkageAction.placeInSlider();
            timer = new Timing.Timer(WAIT_FOR_LINKAGE_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();
        }

        if(Constants.currentSliderActionPos == Constants.SliderActionPos.PLACE_ON_CHAMBER || Constants.currentSliderActionPos == Constants.SliderActionPos.INIT){
            SliderAction.takeFromLinkage();
            timer = new Timing.Timer(WAIT_FOR_SLIDER_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();
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

    public void score(){
        if(Constants.currentScorePos == Constants.ScorePos.CHAMBER){
            sliderController.setTargetPosition(Constants.SLIDER_HIGH_CHAMBER+500);
            timer = new Timing.Timer(350, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){sliderController.update();}timer.pause();

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
