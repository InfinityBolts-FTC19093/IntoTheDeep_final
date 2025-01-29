package org.firstinspires.ftc.teamcode.systems.linkage;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;

import java.util.concurrent.TimeUnit;

public class rotateClaw_controller{

    Timing.Timer timer;

    double targetPosition;

    private Servo claw;

    public rotateClaw_controller(Servo claw){
        this.claw = claw;
    }

    public void setPos(double targetPosition){
        this.targetPosition = targetPosition;
    }

    public void rotate(){
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        if(Constants.currentRotatePos == Constants.RotatePos.HORIZONTAL || Constants.currentRotatePos == Constants.RotatePos.INTER){
            setPos(Constants.ROTATE_VER);
            timer = new Timing.Timer(50,TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
            Constants.currentRotatePos = Constants.RotatePos.VERTICAL;
        } else {
            setPos(Constants.ROTATE_HRZ);
            timer = new Timing.Timer(50,TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
            Constants.currentRotatePos = Constants.RotatePos.HORIZONTAL;
        }
    }

    public void midRotate() {
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        setPos(Constants.ROTATE_INTER);
        timer = new Timing.Timer(50,TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        Constants.currentRotatePos = Constants.RotatePos.INTER;
    }

    public void update(){
        claw.setPosition(targetPosition);
    }
}