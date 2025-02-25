package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;

import java.util.concurrent.TimeUnit;

public class clawRotate_controller {

    Timing.Timer timer;

    double targetPosition;

    private final Servo claw;

    public clawRotate_controller(Servo claw) {
        this.claw = claw;
    }

    public void setPos(double targetPosition){
        this.targetPosition = targetPosition;
    }

    public void V_H(){
        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        if(Constants.currentClawRotatePos == Constants.ClawRotatePos.HORIZONTAL){
            Constants.currentClawRotatePos = Constants.ClawRotatePos.VERTICAL;
            timer = new Timing.Timer(75,TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        } else {
            Constants.currentClawRotatePos = Constants.ClawRotatePos.HORIZONTAL;
            timer = new Timing.Timer(75, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        }
    }

    public void update(){
        if(Constants.currentClawRotatePos == Constants.ClawRotatePos.HORIZONTAL){
            claw.setPosition(Constants.ROTATE_TAKE_HORIONTAL);
        }

        if(Constants.currentClawRotatePos == Constants.ClawRotatePos.VERTICAL){
            claw.setPosition(Constants.ROTATE_TAKE_VERTICAL);

        }

        if(Constants.currentClawRotatePos == Constants.ClawRotatePos.INVERTED){
            claw.setPosition(Constants.ROTATE_PLACE_IN_SLIDER_INVERTED);
        }
    }
}