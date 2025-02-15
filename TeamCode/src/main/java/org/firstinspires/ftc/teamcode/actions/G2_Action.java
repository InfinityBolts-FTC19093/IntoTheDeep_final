package org.firstinspires.ftc.teamcode.actions;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.RobotMap;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

import java.util.concurrent.TimeUnit;

public class G2_Action {

    Timing.Timer timer;
    Servo claw, slider_claw, claw_tilt, claw_rotate, turret, linkage, slider_claw_rotate, unghi_robot;
    DcMotorEx slider;
    slider_controller sliderController;

    public G2_Action(Servo claw, Servo slider_claw, Servo claw_tilt, Servo claw_rotate, Servo turret, Servo linkage, Servo slider_claw_rotate, DcMotorEx slider, Servo unghi_robot) {
        this.claw = claw;
        this.slider_claw = slider_claw;
        this.claw_tilt = claw_tilt;
        this.claw_rotate = claw_rotate;
        this.turret = turret;
        this.linkage = linkage;
        this.slider_claw_rotate = slider_claw_rotate;
        this.slider = slider;
        this.sliderController = new slider_controller(this.slider);
        this.unghi_robot = unghi_robot;
    }

    public void zeroPos(){
        claw.setPosition(Constants.OPEN_CLAW);
        slider_claw.setPosition(Constants.OPEN_CLAW);

        claw_tilt.setPosition(Constants.TILT_INIT);
        claw_rotate.setPosition(Constants.ROTATE_INIT);

        turret.setPosition(Constants.CLAW_ASSEMBLY_INIT);

        linkage.setPosition(Constants.LINKAGE_INIT_POS);

        sliderController.setTargetPosition(Constants.SLIDER_DOWN);
        timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while(!timer.done());timer.pause();

        slider_claw_rotate.setPosition(Constants.TURRET_INIT);
        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while(!timer.done());timer.pause();

        claw_tilt.setPosition(Constants.TILT_INIT);
    }

    public void lev2Asent(){
        sliderController.setTargetPosition(Constants.SLIDER_ASCEND);
        timer = new Timing.Timer(100,TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){sliderController.update();}timer.pause();
        unghi_robot.setPosition(Constants.UNGHI_ROBOT_JOS);
        timer = new Timing.Timer(200, TimeUnit.MILLISECONDS);timer.start();while(!timer.done()){sliderController.update();}timer.pause();
        sliderController.setTargetPosition(Constants.SLIDER_LEV2_ASCEND);
    }
}