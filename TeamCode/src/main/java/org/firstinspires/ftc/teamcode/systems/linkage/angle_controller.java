package org.firstinspires.ftc.teamcode.systems.linkage;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;

import java.util.concurrent.TimeUnit;

public class angle_controller{

    Timing.Timer timer;

    double targetPosition;

    private Servo claw;

    public angle_controller(Servo claw){
        this.claw = claw;
    }

    public void setPos(double targetPosition){
        this.targetPosition = targetPosition;
    }

    public void changeAngle(){
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        if(Constants.currentAnglePos == Constants.AnglePos.INIT || Constants.currentAnglePos == Constants.AnglePos.GIVE){
            setPos(Constants.ANGLE_TAKE);
            timer = new Timing.Timer(50,TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
            Constants.currentAnglePos = Constants.AnglePos.TAKE;
        } else {
            setPos(Constants.ANGLE_INIT);
            timer = new Timing.Timer(50,TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
            Constants.currentAnglePos = Constants.AnglePos.INIT;
        }
    }

    public void giveAngle() {
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        setPos(Constants.ANGLE_GIVE_CLAW);
        Constants.currentAnglePos = Constants.AnglePos.GIVE;
    }

    public void update(){
        claw.setPosition(targetPosition);
    }
}