package org.firstinspires.ftc.teamcode.systems;


import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;

import java.util.concurrent.TimeUnit;

public class sliderClaw_controller {

    Timing.Timer timer;

    double targetPosition;

    private final Servo claw;

    public sliderClaw_controller(Servo claw) {
        this.claw = claw;
    }

    public void setPos(double targetPosition){
        this.targetPosition = targetPosition;
    }

    public void open_close(){
        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        if(Constants.currentSliderClawPos == Constants.SliderClawPos.CLOSE_CLAW){
            Constants.currentSliderClawPos = Constants.SliderClawPos.OPEN_CLAW;
            timer = new Timing.Timer(75,TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        } else {
            Constants.currentSliderClawPos = Constants.SliderClawPos.CLOSE_CLAW;
            timer = new Timing.Timer(75,TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        }
    }

    public void update(){
        if(Constants.currentSliderClawPos == Constants.SliderClawPos.OPEN_CLAW){
            claw.setPosition(Constants.OPEN_CLAW_SLIDER);
        }

        if(Constants.currentSliderClawPos == Constants.SliderClawPos.CLOSE_CLAW){
            claw.setPosition(Constants.CLOSE_CLAW_SLIDER);
        }
    }
}