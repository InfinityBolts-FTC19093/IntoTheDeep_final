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
    private Servo tilt, linkage, claw_rotate, claw, rotate_assembly;
    Timing.Timer timer;
    autoClaw_controller autoClawController = new autoClaw_controller();

    public servo_linkage_action(Servo claw, Servo tilt, Servo linkage, Servo claw_rotate, Servo rotate_assembly){
        this.tilt = tilt;
        this.linkage = linkage;
        this.claw_rotate = claw_rotate;
        this.claw = claw;
        this.rotate_assembly = rotate_assembly;
    }

    public void takePos(){
        linkage.setPosition(Constants.LINKAGE_TAKE_POS);

        claw.setPosition(Constants.OPEN_CLAW);
        rotate_assembly.setPosition(Constants.CLAW_ASSEMBLY_TAKE);
        tilt.setPosition(Constants.TILT_TAKE);

        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        autoClawController.take();

        timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        Constants.currentLinkageActionPos = Constants.LinkageActionPos.TAKE;
    }

    public void plaseInSlider(){
        if(Constants.currentClawPos == Constants.ClawPos.OPEN_CLAW){
            claw.setPosition(Constants.CLOSE_CLAW);
            timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
        }

        claw_rotate.setPosition(Constants.ROTATE_PLACE_IN_SLIDER);

        tilt.setPosition(Constants.TILT_PLACE_IN_SLIDER);
        rotate_assembly.setPosition(Constants.CLAW_ASSEMBLY_PLACE_IN_SLIDER);

        timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

        linkage.setPosition(Constants.LINKAGE_PLACE_IN_SLIDER);

        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

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
