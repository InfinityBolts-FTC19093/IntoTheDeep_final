package org.firstinspires.ftc.teamcode.actions;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.systems.linkage_controller;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

import java.util.concurrent.TimeUnit;

public class G2_Action {

    Timing.Timer timer;
    Servo claw, slider_claw, claw_tilt, claw_rotate, claw_pivot, linkage, turret, slider_tilt;
    DcMotorEx slider;
    static slider_controller sliderController;
    static linkage_controller linkageController;

    public G2_Action(Servo claw, Servo slider_claw, Servo claw_tilt, Servo claw_rotate, Servo claw_pivot, Servo linkage, Servo turret, DcMotorEx slider, Servo slider_tilt) {
        this.claw = claw;
        this.slider_claw = slider_claw;
        this.claw_tilt = claw_tilt;
        this.claw_rotate = claw_rotate;
        this.claw_pivot = claw_pivot;
        this.linkage = linkage;
        this.turret = turret;
        this.slider = slider;
        this.slider_tilt = slider_tilt;
    }

    public static void setSliderController(slider_controller controller){
        sliderController = controller;
    }

    public static void setLinkageController(linkage_controller controller){
        linkageController = controller;
    }

    public void zeroPos(){
        claw.setPosition(Constants.OPEN_CLAW);
        slider_claw.setPosition(Constants.OPEN_CLAW_SLIDER);
        Constants.currentClawPos = Constants.ClawPos.OPEN_CLAW;
        Constants.currentSliderClawPos = Constants.SliderClawPos.OPEN_CLAW;

        claw_tilt.setPosition(Constants.TILT_INIT);
        claw_rotate.setPosition(Constants.ROTATE_INIT);
        Constants.currentClawRotatePos = Constants.ClawRotatePos.HORIZONTAL;

        claw_pivot.setPosition(Constants.CLAW_ASSEMBLY_INIT);

        Constants.currentLinkagePos = Constants.LinkagePos.AUTO;
        linkage.setPosition(Constants.LINKAGE_INIT_POS);
        Constants.currentLinkageActionPos = Constants.LinkageActionPos.INIT;
        linkageController.getlinkagePos(linkage.getPosition());


        sliderController.setTargetPosition(Constants.SLIDER_DOWN);
        timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while(!timer.done()){}timer.pause();

        turret.setPosition(Constants.TURRET_INIT_AUTO);
        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while(!timer.done()){}timer.pause();

        slider_tilt.setPosition(Constants.SLIDER_TILT_INIT_BASKET);
    }

//    public void lev2Asent(){
//        sliderController.setTargetPosition(Constants.SLIDER_ASCEND);
//        timer = new Timing.Timer(100,TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){inTimer.whileInTimer(); sliderController.update();}timer.pause();
//        base_tilt.setPosition(Constants.UNGHI_ROBOT_JOS);
//        timer = new Timing.Timer(200, TimeUnit.MILLISECONDS);timer.start();while(!timer.done()){inTimer.whileInTimer(); sliderController.update();}timer.pause();
//        sliderController.setTargetPosition(Constants.SLIDER_LEV2_ASCEND);
//    }

    public void switchBasketPos(){
        if(Constants.currentBasketPos == Constants.BasketPos.HIGH_BASKET){
            Constants.currentBasketPos = Constants.BasketPos.LOW_BASKET;
            timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while(!timer.done()){sliderController.update();}timer.pause();
        }else{
            Constants.currentBasketPos = Constants.BasketPos.HIGH_BASKET;
            timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while(!timer.done()){sliderController.update();}timer.pause();
        }
    }

    public void reset_slider(){
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slider.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}