package org.firstinspires.ftc.teamcode.actions;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.systems.claw_controller;
import org.firstinspires.ftc.teamcode.systems.linkage_controller;

import java.util.concurrent.TimeUnit;

public class Collect {
    Timing.Timer timer;

    Servo tilt, linkage, claw_rotate, claw, claw_pivot;
    DcMotorEx leftFront, leftBack, rightFront, rightBack;
    ColorSensor rotateSensor, centerSensor;
    Gamepad gamepad1;
    double lim;
    claw_controller clawController;
    static linkage_controller linkageController;
    static robot_drive drive;

    public Collect(Servo claw, Servo tilt, Servo linkage, Servo claw_rotate, Servo claw_pivot, ColorSensor rotateSensor, ColorSensor centerSensor, DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, double lim, Gamepad gamepad1){
        this.tilt = tilt;
        this.linkage = linkage;
        this.claw_rotate = claw_rotate;
        this.claw = claw;
        this.claw_pivot = claw_pivot;
        this.clawController = new claw_controller(this.claw);

        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
        this.gamepad1 = gamepad1;
        this.lim = lim;

        this.rotateSensor = rotateSensor;
        this.centerSensor = centerSensor;
    }

    public static void setLinkageController(linkage_controller controller){
        linkageController = controller;
    }

    public static void setDrive(robot_drive drive1){
        drive = drive1;
    }

    public void whileInTimer(){
        drive.robotCentricDrive(leftFront, leftBack, rightFront, rightBack, lim, gamepad1);
    }
    //(centerSensor.alpha() >= 350 || rotateSensor.alpha() >= 350 ) && rotateSensor.alpha() >= 350
    //centerSensor.alpha() >= 350 && rotateSensor.alpha() <= 350
    public boolean colorDetection1(){
        if(((centerSensor.red()>=250 || centerSensor.blue() >=350 || centerSensor.green()>=350) || (rotateSensor.red()>=250 || rotateSensor.blue() >=350 || rotateSensor.green()>=350)) && (rotateSensor.red()>=250 || rotateSensor.blue() >=350 || rotateSensor.green()>=350)){
            return true;
        }

        return false;
    }

    public boolean colorDetection2(){
        if((centerSensor.red()>=250 || centerSensor.blue() >=350 || centerSensor.green()>=350) && (rotateSensor.red()<=250 || rotateSensor.blue() <=350 || rotateSensor.green()<=350)){
            return true;
        }
        return false;
    }

    public void takePos(){
        Constants.currentLinkagePos = Constants.LinkagePos.AUTO;
        linkage.setPosition(Constants.LINKAGE_TAKE_POS);
        linkageController.getlinkagePos(linkage.getPosition());

        claw.setPosition(Constants.OPEN_CLAW);
        Constants.currentClawPos = Constants.ClawPos.OPEN_CLAW;

        claw_rotate.setPosition(Constants.ROTATE_TAKE_HORIONTAL);
        Constants.currentClawRotatePos = Constants.ClawRotatePos.HORIZONTAL;

        claw_pivot.setPosition(Constants.CLAW_ASSEMBLY_TAKE);
        tilt.setPosition(Constants.TILT_BEFORE_TAKE);

        Constants.currentLinkageActionPos = Constants.LinkageActionPos.TAKE;
    }

    public void placeInSlider(){
        tilt.setPosition(Constants.TILT_TAKE);
        timer = new Timing.Timer(175, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();

        if(Constants.currentClawPos == Constants.ClawPos.OPEN_CLAW){
            claw.setPosition(Constants.CLOSE_CLAW);
            Constants.currentClawPos = Constants.ClawPos.CLOSE_CLAW;
            timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();
        }

        tilt.setPosition(Constants.TILT_AFTER_TAKE);
        timer = new Timing.Timer(250, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();

        if(colorDetection1()){
            tilt.setPosition(Constants.TILT_PLACE_IN_SLIDER);
            timer = new Timing.Timer(75, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();

            claw_rotate.setPosition(Constants.ROTATE_TAKE_HORIONTAL);
            Constants.currentClawRotatePos = Constants.ClawRotatePos.HORIZONTAL;

            Constants.currentLinkagePos = Constants.LinkagePos.AUTO;
            linkage.setPosition(Constants.LINKAGE_PLACE_IN_SLIDER);
            linkageController.getlinkagePos(linkage.getPosition());

            Constants.currentLinkageActionPos = Constants.LinkageActionPos.PLACE_IN_SLIDER;
        }else if(colorDetection2()){
            tilt.setPosition(Constants.TILT_PLACE_IN_SLIDER);
            timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();

            claw_rotate.setPosition(Constants.ROTATE_PLACE_IN_SLIDER_INVERTED);
            Constants.currentClawRotatePos = Constants.ClawRotatePos.INVERTED;
            timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();

            Constants.currentLinkagePos = Constants.LinkagePos.AUTO;
            linkage.setPosition(Constants.LINKAGE_PLACE_IN_SLIDER);
            linkageController.getlinkagePos(linkage.getPosition());

            Constants.currentLinkageActionPos = Constants.LinkageActionPos.PLACE_IN_SLIDER;
        }else{
            tilt.setPosition(Constants.TILT_BEFORE_TAKE);
            claw.setPosition(Constants.OPEN_CLAW);
            claw_rotate.setPosition(Constants.ROTATE_TAKE_HORIONTAL);
            Constants.currentClawRotatePos = Constants.ClawRotatePos.HORIZONTAL;
            Constants.currentClawPos = Constants.ClawPos.OPEN_CLAW;
        }
    }

    public void placeInSliderWithoutCollect() {
        claw_rotate.setPosition(Constants.ROTATE_PLACE_IN_SLIDER);
        Constants.currentClawRotatePos = Constants.ClawRotatePos.HORIZONTAL;
        timer = new Timing.Timer(175, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()) {whileInTimer();}timer.pause();

        tilt.setPosition(Constants.TILT_PLACE_IN_SLIDER);

        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()) {whileInTimer();}timer.pause();

        Constants.currentLinkagePos = Constants.LinkagePos.AUTO;
        linkage.setPosition(Constants.LINKAGE_PLACE_IN_SLIDER);
        linkageController.getlinkagePos(linkage.getPosition());

        Constants.currentLinkageActionPos = Constants.LinkageActionPos.PLACE_IN_SLIDER;
    }

    public void placeInObservation(){
        tilt.setPosition(Constants.TILT_BEFORE_TAKE);

        claw_pivot.setPosition(Constants.CLAW_ASSEMBLY_PLACE_IN_SLIDER);
        Constants.currentLinkagePos = Constants.LinkagePos.AUTO;
        linkage.setPosition(Constants.LINKAGE_TAKE_POS);
        linkageController.getlinkagePos(linkage.getPosition());
        Constants.currentLinkageActionPos = Constants.LinkageActionPos.TAKE;

        timer = new Timing.Timer(400, TimeUnit.MILLISECONDS); timer.start(); while (!timer.done());{whileInTimer();} timer.pause();
        Constants.currentClawPos = Constants.ClawPos.OPEN_CLAW;
        claw.setPosition(Constants.OPEN_CLAW);

        claw_rotate.setPosition(Constants.ROTATE_PLACE_IN_SLIDER);
        Constants.currentClawRotatePos = Constants.ClawRotatePos.HORIZONTAL;

        tilt.setPosition(Constants.TILT_PLACE_IN_SLIDER);
        claw_pivot.setPosition(Constants.CLAW_ASSEMBLY_PLACE_IN_SLIDER);

        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();

        Constants.currentLinkagePos = Constants.LinkagePos.AUTO;
        linkage.setPosition(Constants.LINKAGE_PLACE_IN_SLIDER);
        linkageController.getlinkagePos(linkage.getPosition());

        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();

        Constants.currentLinkageActionPos = Constants.LinkageActionPos.PLACE_IN_SLIDER;
    }

    public void placeInObservation2(){
        tilt.setPosition(Constants.TILT_BEFORE_TAKE);

        claw_pivot.setPosition(Constants.CLAW_ASSEMBLY_PLACE_IN_SLIDER);

        linkageController.manualControl(gamepad1);

        timer = new Timing.Timer(400, TimeUnit.MILLISECONDS); timer.start(); while (!timer.done());{whileInTimer();} timer.pause();
        Constants.currentClawPos = Constants.ClawPos.OPEN_CLAW;
        claw.setPosition(Constants.OPEN_CLAW);

        claw_rotate.setPosition(Constants.ROTATE_PLACE_IN_SLIDER);
        Constants.currentClawRotatePos = Constants.ClawRotatePos.HORIZONTAL;

        tilt.setPosition(Constants.TILT_PLACE_IN_SLIDER);
        claw_pivot.setPosition(Constants.CLAW_ASSEMBLY_PLACE_IN_SLIDER);

        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();

        Constants.currentLinkagePos = Constants.LinkagePos.AUTO;
        linkage.setPosition(Constants.LINKAGE_PLACE_IN_SLIDER);
        linkageController.getlinkagePos(linkage.getPosition());

        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){whileInTimer();}timer.pause();

        Constants.currentLinkageActionPos = Constants.LinkageActionPos.PLACE_IN_SLIDER;
    }

    public void switchServoAction_TakeSlider(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.PLACE_IN_SLIDER || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start(); while (!timer.done()){whileInTimer();}timer.pause();
            takePos();
        }else if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE || Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE2){
            timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start(); while (!timer.done()){whileInTimer();}timer.pause();
            placeInSlider();
        }
    }
}