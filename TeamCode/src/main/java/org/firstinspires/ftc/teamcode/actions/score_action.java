package org.firstinspires.ftc.teamcode.actions;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.constants.Constants.WAIT_FOR_LINKAGE_ACTION;
import static org.firstinspires.ftc.teamcode.constants.Constants.WAIT_FOR_SLIDER_ACTION;

import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.RobotMap;
import org.firstinspires.ftc.teamcode.systems.sliderClaw_controller;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

import java.util.concurrent.TimeUnit;

public class score_action {

    Timing.Timer timer;
    RobotMap robot = new RobotMap(hardwareMap);

    servo_linkage_action servoLinkageAction = new servo_linkage_action(robot.claw ,robot.claw_tilt, robot.linkage, robot.claw_rotate, robot.rotate_claw_assembly);
    servo_slider_action servoSliderAction =new servo_slider_action();

    sliderClaw_controller sliderClawController = new sliderClaw_controller(robot.slider_claw);
    slider_controller sliderController = new slider_controller(robot.slider);

    public score_action() {}

    /** CHAMBER */

    public void placeOnHighChamber(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            servoLinkageAction.plaseInSlider();
            timer = new Timing.Timer(WAIT_FOR_LINKAGE_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        }

        if(Constants.currentSliderActionPos == Constants.SliderActionPos.PLACE_ON_CHAMBER || Constants.currentSliderActionPos == Constants.SliderActionPos.INIT){
            servoSliderAction.takeFromLinkage();
            timer = new Timing.Timer(WAIT_FOR_SLIDER_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        }

        sliderController.setTargetPosition(Constants.SLIDER_HIGH_CHAMBER);
        servoSliderAction.placeOnHighChamber();
        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        sliderController.setTargetPosition(Constants.SLIDER_PLACE_ON_CHAMBER);
        sliderController.update();
        timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        sliderClawController.setPos(Constants.OPEN_CLAW);

        Constants.currentScorePos = Constants.ScorePos.PLACE_ON_HIGH_CHAMBER;
    }

    public void placeOnLowChamber(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            servoLinkageAction.plaseInSlider();
            timer = new Timing.Timer(WAIT_FOR_LINKAGE_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        }

        if(Constants.currentSliderActionPos == Constants.SliderActionPos.PLACE_ON_CHAMBER || Constants.currentSliderActionPos == Constants.SliderActionPos.INIT){
            servoSliderAction.takeFromLinkage();
            timer = new Timing.Timer(WAIT_FOR_SLIDER_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        }

        sliderController.setTargetPosition(Constants.SLIDER_LOW_CHAMBER);
        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        sliderController.setTargetPosition(Constants.SLIDER_PLACE_ON_CHAMBER);
        sliderClawController.setPos(Constants.OPEN_CLAW);

        Constants.currentScorePos = Constants.ScorePos.PLACE_ON_LOW_CHAMBER;
    }


    /** BUSKET */

    public void placeInHighBusket(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            servoLinkageAction.plaseInSlider();
            timer = new Timing.Timer(WAIT_FOR_LINKAGE_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        }
        //add slider servo
        sliderController.setTargetPosition(Constants.SLIDER_HIGH_BUSKET);

        Constants.currentScorePos = Constants.ScorePos.PLACE_ON_HIGH_BUSKET;
    }



    public void placeInLowBusket(){
        if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
            servoLinkageAction.plaseInSlider();
            timer = new Timing.Timer(WAIT_FOR_LINKAGE_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        }
        //add slider servo
        sliderController.setTargetPosition(Constants.SLIDER_LOW_BUSKET);

        Constants.currentScorePos = Constants.ScorePos.PLACE_ON_LOW_BUSKET;
    }
}
