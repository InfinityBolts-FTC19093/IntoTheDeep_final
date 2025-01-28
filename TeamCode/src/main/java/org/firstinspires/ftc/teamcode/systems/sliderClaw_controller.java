package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;

import java.util.concurrent.TimeUnit;

public class sliderClaw_controller {
    Timing.Timer timer;

    double targetPosition;

    private final Servo claw;

    public sliderClaw_controller(Servo claw) {this.claw = claw;}

    public void setPos(double targetPosition){
        this.targetPosition = targetPosition;
    }

    public void open_close(){
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        if(Constants.currentClawPos == Constants.ClawPos.CLOSE_CLAW){
            setPos(Constants.OPEN_CLAW);
            timer = new Timing.Timer(50,TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
            Constants.currentClawPos = Constants.ClawPos.OPEN_CLAW;
        } else {
            setPos(Constants.CLOSE_CLAW);
            timer = new Timing.Timer(50,TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
            Constants.currentClawPos = Constants.ClawPos.CLOSE_CLAW;
        }
    }

    public void update(){
        claw.setPosition(targetPosition);
    }
}