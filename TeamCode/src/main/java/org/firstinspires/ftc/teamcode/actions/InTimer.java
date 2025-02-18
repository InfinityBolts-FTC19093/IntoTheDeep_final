package org.firstinspires.ftc.teamcode.actions;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.systems.clawRotate_controller;
import org.firstinspires.ftc.teamcode.systems.claw_controller;
import org.firstinspires.ftc.teamcode.systems.linkage_controller;
import org.firstinspires.ftc.teamcode.systems.manualSlider_controller;
import org.firstinspires.ftc.teamcode.systems.sliderClaw_controller;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

public class InTimer {
    DcMotorEx leftFront, leftBack, rightFront, rightBack, slider;double lim;
    Gamepad gamepad1;
    robot_drive drive;


    public InTimer(DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, double lim, Gamepad gamepad1){
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
        this.lim = lim;
        this.gamepad1 = gamepad1;

        this.drive = new robot_drive(leftFront, leftBack, rightFront, rightBack, 1, gamepad1);
    }

    public void whileInTimer(){
        drive.robotCentricDrive(leftFront, leftBack, rightFront, rightBack, 1, gamepad1);
    }
}
