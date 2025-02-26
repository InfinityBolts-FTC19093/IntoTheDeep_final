package org.firstinspires.ftc.teamcode.actions;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class InTimer {
    DcMotorEx leftFront, leftBack, rightFront, rightBack, slider;double lim;
    Gamepad gamepad1;
    static robot_drive drive;



    public InTimer(DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, double lim, Gamepad gamepad1){
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
        this.lim = lim;
        this.gamepad1 = gamepad1;
    }

    public static void setRobotDrive(robot_drive drive1){
        drive = drive1;
    }

    public void whileInTimer(){
        drive.robotCentricDrive(leftFront, leftBack, rightFront, rightBack, 1, gamepad1);
    }
}
