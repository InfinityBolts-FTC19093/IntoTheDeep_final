package org.firstinspires.ftc.teamcode.actions;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.RobotMap;
import org.firstinspires.ftc.teamcode.systems.slider.slider_controller;

import java.util.concurrent.TimeUnit;

public class servo_slider_action {
    Timing.Timer timer;

    RobotMap robot = new RobotMap(hardwareMap);

    slider_controller sliderController = new slider_controller(robot.slider);

    public servo_slider_action(){}

    public void takeFromLinkage(){
        if(Constants.currentSliderClawPos == Constants.SliderClawPos.CLOSE_CLAW){
            robot.slider_claw.setPosition(Constants.SLIDER_OPEN);
            timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        }

        sliderController.setTargetPosition(Constants.SLIDER_TAKE_FORM_LINKAGE);
        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        robot.slider_claw_rotate.setPosition(Constants.SLIDER_ROTATE_TAKE_FROM_LINKAGE);
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        robot.slider_claw_tilt.setPosition(Constants.SLIDER_TILT_TAKE_FROM_LINKAGE);
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        robot.slider_claw.setPosition(Constants.SLIDER_CLOSE);
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        Constants.currentSliderActionPos = Constants.SliderActionPos.TAKE_FOR_LINKAGE;
    }

    public void placeOnHighChamber(){
        robot.slider_claw_tilt.setPosition(Constants.SLIDER_TILT_PLACE_ON_HIGH_CHAMBER);
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        robot.slider_claw_rotate.setPosition(Constants.SLIDER_ROTATE_PLACE);
        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        sliderController.setTargetPosition(Constants.SLIDER_HIGH_CHAMBER);
        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        sliderController.setTargetPosition(Constants.SLIDER_PLACE_ON_CHAMBER);
    }

    public void placeOnLowChamber(){

    }

    public void placeOnHighBusket(){

    }

    public void placeOnLowBusket(){

    }
}
