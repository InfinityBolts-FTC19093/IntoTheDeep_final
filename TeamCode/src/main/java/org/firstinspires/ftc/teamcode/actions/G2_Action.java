package org.firstinspires.ftc.teamcode.actions;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.RobotMap;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

import java.util.concurrent.TimeUnit;

public class G2_Action {

    Timing.Timer timer;


    RobotMap robot;

    public G2_Action(HardwareMap hardwareMap){
        robot = new RobotMap(hardwareMap);
    }

    slider_controller sliderController = new slider_controller(robot.slider);

    public void zeroPos(){
        robot.claw.setPosition(Constants.OPEN_CLAW);
        robot.slider_claw.setPosition(Constants.SLIDER_OPEN);

        robot.claw_tilt.setPosition(Constants.TILT_INIT);
        robot.claw_rotate.setPosition(Constants.ROTATE_INIT);

        robot.rotate_claw_assembly.setPosition(Constants.CLAW_ASSEMBLY_INIT);

        robot.linkage.setPosition(Constants.LINKAGE_INIT_POS);

        sliderController.setTargetPosition(Constants.SLIDER_DOWN);
        timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while(!timer.done());timer.pause();

        robot.slider_claw_rotate.setPosition(Constants.SLIDER_ROTATE_INIT);
        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while(!timer.done());timer.pause();

        robot.claw_tilt.setPosition(Constants.TILT_INIT);
    }

    public void lev2Asent(){
        sliderController.setTargetPosition(Constants.SLIDER_ASCEND);
        timer = new Timing.Timer(100,TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){sliderController.update();}timer.pause();
        robot.unghi_robot.setPosition(Constants.UNGHI_ROBOT_JOS);
        timer = new Timing.Timer(200, TimeUnit.MILLISECONDS);timer.start();while(!timer.done()){sliderController.update();}timer.pause();
        sliderController.setTargetPosition(Constants.SLIDER_LEV2_ASCEND);
    }
}