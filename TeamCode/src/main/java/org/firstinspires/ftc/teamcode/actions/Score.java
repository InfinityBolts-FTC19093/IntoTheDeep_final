package org.firstinspires.ftc.teamcode.actions;

import static org.firstinspires.ftc.teamcode.constants.Constants.WAIT_FOR_LINKAGE_ACTION;
import static org.firstinspires.ftc.teamcode.constants.Constants.WAIT_FOR_SLIDER_ACTION;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.systems.sliderClaw_controller;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

import java.util.concurrent.TimeUnit;

public class Score {

    private Timing.Timer timer;
    private final Servo claw, claw_tilt, linkage, claw_rotate, turret, slider_claw, slider_claw_tilt, slider_claw_rotate;
    private final DcMotorEx slider;
    private final Collect LinkageAction;
    private final Prepare SliderAction;
    private final slider_controller sliderController;
    private final sliderClaw_controller sliderClawController;

    public Score(Servo claw, Servo claw_tilt, Servo linkage, Servo claw_rotate, Servo turret, Servo slider_claw, Servo slider_claw_tilt, Servo slider_claw_rotate, DcMotorEx slider) {
        this.claw = claw;
        this.claw_tilt = claw_tilt;
        this.linkage = linkage;
        this.claw_rotate = claw_rotate;
        this.turret = turret;
        this.slider_claw = slider_claw;
        this.slider_claw_tilt = slider_claw_tilt;
        this.slider_claw_rotate = slider_claw_rotate;
        this.slider = slider;
        this.LinkageAction = new Collect(this.claw, this.claw_tilt, this.linkage, this.claw_rotate, this.turret);
        this.SliderAction = new Prepare(this.slider_claw, this.slider_claw_tilt, this.slider_claw_rotate, this.slider);
        this.sliderClawController = new sliderClaw_controller(this.slider_claw);
        this.sliderController = new slider_controller(this.slider);
    }

    /** CHAMBER */

    public void placeOnHighChamber(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            LinkageAction.placeInSlider();
            timer = new Timing.Timer(WAIT_FOR_LINKAGE_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        }

        if(Constants.currentSliderActionPos == Constants.SliderActionPos.PLACE_ON_CHAMBER || Constants.currentSliderActionPos == Constants.SliderActionPos.INIT){
            SliderAction.takeFromLinkage();
            timer = new Timing.Timer(WAIT_FOR_SLIDER_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        }

        SliderAction.placeOnHighChamber();
        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        Constants.currentScorePos = Constants.ScorePos.CHAMBER;
    }

    public void placeOnLowChamber(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            LinkageAction.placeInSlider();
            timer = new Timing.Timer(WAIT_FOR_LINKAGE_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        }

        if(Constants.currentSliderActionPos == Constants.SliderActionPos.PLACE_ON_CHAMBER || Constants.currentSliderActionPos == Constants.SliderActionPos.INIT){
            SliderAction.takeFromLinkage();
            timer = new Timing.Timer(WAIT_FOR_SLIDER_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        }

        sliderController.setTargetPosition(Constants.SLIDER_LOW_CHAMBER);
        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        sliderController.setTargetPosition(Constants.SLIDER_LOW_CHAMBER-200);
        sliderClawController.setPos(Constants.OPEN_CLAW);

        Constants.currentScorePos = Constants.ScorePos.CHAMBER;
    }


    /** BUSKET */

    public void placeInHighBusket(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            LinkageAction.placeInSlider();
            timer = new Timing.Timer(WAIT_FOR_LINKAGE_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        }
        SliderAction.placeOnHighBusket();

        Constants.currentScorePos = Constants.ScorePos.BUSKET;
    }



    public void placeInLowBusket(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            LinkageAction.placeInSlider();
            timer = new Timing.Timer(WAIT_FOR_LINKAGE_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        }
        //add slider servo
        sliderController.setTargetPosition(Constants.SLIDER_LOW_BUSKET);

        Constants.currentScorePos = Constants.ScorePos.BUSKET;
    }

    public void placeSample(){
        if(Constants.currentScorePos == Constants.ScorePos.CHAMBER){
            sliderController.setTargetPosition(Constants.SLIDER_HIGH_CHAMBER-200);
            timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){sliderController.update();}timer.pause();

            sliderClawController.setPos(Constants.OPEN_CLAW);
        }
    }

}
