package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;

import java.util.concurrent.TimeUnit;

public class linkage_controller {
    Timing.Timer timer;

    double targetPosition;

    private Servo linkage;

    public linkage_controller(Servo linkage){
        this.linkage = linkage;
    }

    public void setPos(double targetPosition){this.targetPosition = targetPosition;}

    public void setTargetPosition(double targetPosition) {
        linkage.setPosition(targetPosition);
    }

    public void update(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE){
            linkage.setPosition(Constants.LINKAGE_TAKE_POS);
        }

        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.PLACE_IN_SLIDER){
            linkage.setPosition(Constants.LINKAGE_PLACE_IN_SLIDER);
        }

        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT || Constants.currentLinkageActionPos == Constants.LinkageActionPos.OSERVATION){
            linkage.setPosition(Constants.LINKAGE_INIT_POS);
        }
    }
}
