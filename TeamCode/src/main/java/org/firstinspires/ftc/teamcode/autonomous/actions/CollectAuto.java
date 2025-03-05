package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.systems.claw_controller;
import org.firstinspires.ftc.teamcode.systems.linkage_controller;

public class CollectAuto {
    Timing.Timer timer;
    Servo claw_tilt, linkage, claw_rotate, claw, claw_pivot;
    static linkage_controller linkageController;
    claw_controller clawController;
    ColorSensor rotateSensor, centerSensor;
    public CollectAuto(Servo claw, Servo claw_tilt, Servo linkage, Servo claw_rotate, Servo claw_pivot, ColorSensor rotateSensor, ColorSensor centerSensor){
        this.claw_tilt = claw_tilt;
        this.linkage = linkage;
        this.claw_rotate = claw_rotate;
        this.claw = claw;
        this.claw_pivot = claw_pivot;
        this.rotateSensor = rotateSensor;
        this.centerSensor = centerSensor;
        this.clawController = new claw_controller(this.claw);
    }

    public static void setLinkageController(linkage_controller controller){
        linkageController = controller;
    }

    public void takePos () {
        claw.setPosition(Constants.OPEN_CLAW);
        linkage.setPosition(Constants.LINKAGE_TAKE_POS);
        claw_tilt.setPosition(Constants.TILT_BEFORE_TAKE);
        claw_pivot.setPosition(Constants.CLAW_PIVOT_TAKE);
        claw_rotate.setPosition(Constants.ROTATE_TAKE_VERTICAL);
    }

    public void LeaveSample() {
        claw.setPosition(Constants.OPEN_CLAW);
    }

    public void TakeGround() {
        claw_tilt.setPosition(Constants.TILT_TAKE);
        new SleepAction(.1);
        claw.setPosition(Constants.CLOSE_CLAW);
    }

    public void placeInSlider () {
        claw.setPosition(Constants.CLOSE_CLAW);
        new SleepAction(.1);
        claw_rotate.setPosition(Constants.ROTATE_INIT);
        claw_tilt.setPosition(Constants.TILT_PLACE_IN_SLIDER);
        claw_pivot.setPosition(Constants.CLAW_ASSEMBLY_PLACE_IN_SLIDER);
    }
}