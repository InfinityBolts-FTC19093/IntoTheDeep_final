package org.firstinspires.ftc.teamcode.actions;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.RobotMap;
import org.firstinspires.ftc.teamcode.systems.autoClaw_controller;
import org.firstinspires.ftc.teamcode.systems.claw_controller;

import java.util.concurrent.TimeUnit;

public class servo_linkage_action {
    private Servo tilt, linkage;
    Timing.Timer timer;

    RobotMap robot = new RobotMap(hardwareMap);
    claw_controller clawController = new claw_controller(robot.claw);
    autoClaw_controller autoClawController = new autoClaw_controller();

    public servo_linkage_action(Servo tilt, Servo linkage){
        this.tilt = tilt;
        this.linkage = linkage;
    }

    public void takePos(){
        clawController.setPos(Constants.OPEN_CLAW);

        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        autoClawController.take();

        timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        tilt.setPosition(Constants.TILT_TAKE);
        linkage.setPosition(Constants.LINKAGE_TAKE_POS);
        Constants.currentLinkageActionPos = Constants.LinkageActionPos.TAKE;
    }

    public void plaseInSlider(){
        clawController.setPos(Constants.CLOSE_CLAW);

        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        autoClawController.placeInSlider();

        timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        tilt.setPosition(Constants.TILT_PLACE_IN_SLIDER);
        linkage.setPosition(Constants.LINKAGE_PLACE_IN_SLIDER);
        Constants.currentLinkageActionPos = Constants.LinkageActionPos.PLACE_IN_SLIDER;
    }

    public void switchServoAction(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.PLACE_IN_SLIDER){
            takePos();
        }else if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE){
            plaseInSlider();
        }
    }
}
