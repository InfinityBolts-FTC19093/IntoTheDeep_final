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
            linkage.setPosition(pos);
        }

        if(Constants.currentLinkagePos == Constants.LinkagePos.MANUAL){
            linkage.setPosition(manualPos);
        }
    }

    public void manualControl(double leftTrigger, double rightTrigger){
        if(leftTrigger >=0.01 || rightTrigger >=0.01){
            if (leftTrigger>= 0.05) {
                manualPos -=0.05;
                linkage.setPosition(manualPos);
                Constants.currentLinkagePos = Constants.LinkagePos.MANUAL;
            }
            if (rightTrigger >= 0.05) {
                manualPos += 0.05;
                linkage.setPosition(manualPos);
                Constants.currentLinkagePos = Constants.LinkagePos.MANUAL;
            }
        }else{
            Constants.currentLinkagePos = Constants.LinkagePos.AUTO;
        }
    }
}