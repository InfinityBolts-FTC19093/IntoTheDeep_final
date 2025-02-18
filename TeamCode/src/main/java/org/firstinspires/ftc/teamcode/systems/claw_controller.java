package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;

import java.util.concurrent.TimeUnit;

public class claw_controller{

    Timing.Timer timer;

    double targetPosition;

    private Servo claw;

    public claw_controller(Servo claw){
        this.claw = claw;
    }

    public void setPos(double targetPosition){this.targetPosition = targetPosition;}

    public void setTargetPosition(double targetPosition) {
        claw.setPosition(targetPosition);
    }

    public void open_close(){
        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        if(Constants.currentClawPos == Constants.ClawPos.CLOSE_CLAW){
            Constants.currentClawPos = Constants.ClawPos.OPEN_CLAW;
            timer = new Timing.Timer(75,TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        } else {
            Constants.currentClawPos = Constants.ClawPos.CLOSE_CLAW;
            timer = new Timing.Timer(75,TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        }
    }

    public void update(){
        if(Constants.currentClawPos == Constants.ClawPos.OPEN_CLAW){
            claw.setPosition(Constants.OPEN_CLAW);
        }

        if(Constants.currentClawPos == Constants.ClawPos.CLOSE_CLAW){
            claw.setPosition(Constants.CLOSE_CLAW);
        }
    }
}