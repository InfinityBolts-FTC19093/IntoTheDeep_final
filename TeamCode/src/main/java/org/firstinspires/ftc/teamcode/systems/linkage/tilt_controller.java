package org.firstinspires.ftc.teamcode.systems.linkage;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;

import java.util.concurrent.TimeUnit;

public class tilt_controller{

    Timing.Timer timer;

    double targetPosition;

    private Servo claw;

    public tilt_controller(Servo claw){
        this.claw = claw;
    }

    public void setPos(double targetPosition){
        this.targetPosition = targetPosition;
    }

    public void tiltTake(){
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        if(Constants.currentTiltPos == Constants.TiltPos.INIT || Constants.currentTiltPos == Constants.TiltPos.GIVE){
            setPos(Constants.TILT_TAKE);
            timer = new Timing.Timer(50,TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
            Constants.currentTiltPos = Constants.TiltPos.TAKE;
        } else {
            setPos(Constants.TILT_INIT);
            timer = new Timing.Timer(50,TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
            Constants.currentTiltPos = Constants.TiltPos.INIT;
        }
    }

    public void tiltGive () {
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        setPos(Constants.TILT_PLACE_IN_SLIDER);
        Constants.currentTiltPos = Constants.TiltPos.GIVE;
    }

    public void update(){
        claw.setPosition(targetPosition);
    }
}