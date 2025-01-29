package org.firstinspires.ftc.teamcode.utils.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.RobotMap;
import org.firstinspires.ftc.teamcode.systems.slider.slider_controller;

@TeleOp
public class sliderPos extends LinearOpMode {

    RobotMap robot = new RobotMap(hardwareMap);
    slider_controller sliderController = new slider_controller(robot.slider);

    @Override
    public void runOpMode(){
        telemetry.addData("slider pos" ,sliderController.pos());
        telemetry.update();
    }
}
