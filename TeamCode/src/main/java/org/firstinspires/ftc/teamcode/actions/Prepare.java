package org.firstinspires.ftc.teamcode.actions;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

import java.util.concurrent.TimeUnit;

public class Prepare {
    Timing.Timer timer;

    private final Servo claw, tilt, rotate;
    private DcMotorEx slider;
    private final slider_controller sliderController;

    public Prepare(Servo claw, Servo tilt, Servo rotate, DcMotorEx slider){
        this.claw = claw;
        this.tilt = tilt;
        this.rotate = rotate;
        this.slider = slider;
        this.sliderController = new slider_controller(this.slider);
    }

    //slider_controller sliderController = new slider_controller(slider);
    public void takeFromLinkage(){
        if(Constants.currentSliderClawPos == Constants.SliderClawPos.CLOSE_CLAW){
            claw.setPosition(Constants.SLIDER_OPEN);
            timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        }

        rotate.setPosition(Constants.TURRET_TAKE_FROM_LINKAGE);
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        tilt.setPosition(Constants.SLIDER_TILT_TAKE_FROM_LINKAGE);
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        sliderController.setTargetPosition(Constants.SLIDER_TAKE_FORM_LINKAGE);
        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){sliderController.update();};timer.pause();

        claw.setPosition(Constants.SLIDER_CLOSE);
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        Constants.currentSliderActionPos = Constants.SliderActionPos.TAKE_FOR_LINKAGE;
    }

    public void takeFromHuman(){
        if(Constants.currentSliderClawPos == Constants.SliderClawPos.CLOSE_CLAW){
            claw.setPosition(Constants.SLIDER_OPEN);
            timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        }

        rotate.setPosition(Constants.TURRET_TAKE_HUMAN);
        tilt.setPosition(Constants.SLIDER_TILT_TAKE_FORM_HUMAN);

        sliderController.setTargetPosition(Constants.SLIDER_DOWN);

    }

    public void placeOnHighChamber(){
        tilt.setPosition(Constants.SLIDER_TILT_PLACE_ON_HIGH_CHAMBER);
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        rotate.setPosition(Constants.TURRET_PLACE);
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        sliderController.setTargetPosition(Constants.SLIDER_HIGH_CHAMBER);
        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){sliderController.update();}timer.pause();

        sliderController.setTargetPosition(Constants.SLIDER_HIGH_CHAMBER-200);

        Constants.previousSliderActionPos = Constants.currentSliderActionPos;
        Constants.currentSliderActionPos = Constants.SliderActionPos.PLACE_ON_CHAMBER;
    }

    public void placeOnLowChamber(){

    }

    public void placeOnHighBusket(){
        sliderController.setTargetPosition(Constants.SLIDER_HIGH_BUSKET);
        timer = new Timing.Timer(200, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){sliderController.update();}timer.pause();

        tilt.setPosition(Constants.SLIDER_TILT_PLACE_IN_BUSKET);
        rotate.setPosition(Constants.TURRET_PLACE);

        Constants.currentSliderActionPos = Constants.SliderActionPos.PLACE_IN_BUSKET;
    }

    public void placeOnLowBusket(){

    }

    public void switchSliderAction(){
        if(Constants.currentSliderActionPos == Constants.SliderActionPos.PLACE_IN_BUSKET){
            takeFromLinkage();
        } else if (Constants.currentSliderActionPos == Constants.SliderActionPos.TAKE_FOR_LINKAGE) {
            placeOnHighChamber();
        }
    }
}
