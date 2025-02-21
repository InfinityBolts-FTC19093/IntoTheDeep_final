package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;

import java.util.concurrent.TimeUnit;

public class linkage_controller {
    Timing.Timer timer;

    double targetPosition;

    private Servo linkage;
    private double pos, manualPos = 0.5;

    public linkage_controller(Servo linkage){
        this.linkage = linkage;
    }

    public void getlinkagePos(double Pos){
        pos = Pos;
    }

    public void setPos(double targetPosition){this.targetPosition = targetPosition;}

    public void setTargetPosition(double targetPosition) {
        linkage.setPosition(targetPosition);
    }

    public void update(){
        if(Constants.currentLinkagePos == Constants.LinkagePos.AUTO){
            linkage.scaleRange(0,1);
            linkage.setPosition(pos);
        }

        if(Constants.currentLinkagePos == Constants.LinkagePos.MANUAL){
            linkage.scaleRange(0.1,.79);
            linkage.setPosition(manualPos);
        }
    }

    public void manualControl(){
        Constants.currentLinkagePos = Constants.LinkagePos.MANUAL;
        linkage.scaleRange(0.1,.79);

        if(manualPos == 1){
            manualPos = 0.75;
            linkage.setPosition(manualPos);
        }else if(manualPos == .75){
            manualPos = .5;
            linkage.setPosition(manualPos);
        }else if(manualPos == .5){
            manualPos = .25;
            linkage.setPosition(manualPos);
        }else if(manualPos == .25){
            manualPos = 0;
            linkage.setPosition(manualPos);
        }else if(manualPos == 0){
            manualPos = 1;
            linkage.setPosition(manualPos);

        }
        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS); timer.start(); while (!timer.done()); timer.pause();
    }
}