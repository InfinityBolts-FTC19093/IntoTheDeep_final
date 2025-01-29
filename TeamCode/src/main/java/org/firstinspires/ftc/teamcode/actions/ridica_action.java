package org.firstinspires.ftc.teamcode.actions;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.RobotMap;
import org.firstinspires.ftc.teamcode.systems.slider.slider_controller;

import java.util.concurrent.TimeUnit;

public class ridica_action{

    RobotMap robot = new RobotMap(hardwareMap);
    slider_controller sliderController = new slider_controller(robot.slider);

    Timing.Timer timer;

    public void ridica_action(){}

    public void start(){
        sliderController.setTargetPosition(Constants.SLIDER_ASCEND);
        timer = new Timing.Timer(100,TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        robot.liftRobot.setPosition(Constants.ANGLE_GIVE_CLAW);
        sliderController.setTargetPosition(Constants.SLIDER_DOWN);
    }
}
