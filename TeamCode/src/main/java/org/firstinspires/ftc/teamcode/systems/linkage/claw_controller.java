package org.firstinspires.ftc.teamcode.systems.linkage;

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

    public void setPos(double targetPosition){
        this.targetPosition = targetPosition;
    }

    public void open_close(){
        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        if(Constants.currentClawPos == Constants.ClawPos.CLOSE){
            setPos(Constants.OPEN_CLAW);
            timer = new Timing.Timer(50,TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
            Constants.currentClawPos = Constants.ClawPos.OPEN;
        } else {
            setPos(Constants.CLOSE_CLAW);
            timer = new Timing.Timer(50,TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
            Constants.currentClawPos = Constants.ClawPos.CLOSE;
        }
    }

    public void update(){
        claw.setPosition(targetPosition);
    }
}