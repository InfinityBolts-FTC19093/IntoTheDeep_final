package org.firstinspires.ftc.teamcode.actions;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;


public class robot_drive {
    DcMotorEx leftFront, leftBack, rightFront, rightBack;
    double lim;
    Gamepad gamepad1;
    HardwareMap hardwareMap;

    public robot_drive(DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, double lim, Gamepad gamepad1, HardwareMap hardwareMap){
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
        this.lim = lim;
        this.gamepad1 = gamepad1;
        this.hardwareMap = hardwareMap;
    }

    public void robotCentricDrive(DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, double lim, Gamepad gamepad1) {
        double y = -gamepad1.left_stick_y;
        double x =  gamepad1.left_stick_x* 1;
        double rx = gamepad1.right_stick_x*1;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftPower = Clip(frontLeftPower,lim);
        backLeftPower = Clip(backLeftPower,lim);
        frontRightPower = Clip(frontRightPower,lim);
        backRightPower = Clip(backRightPower,lim);

        leftFront.setPower(frontLeftPower*(12/hardwareMap.getAll(VoltageSensor.class).get(0).getVoltage()));
        leftBack.setPower(backLeftPower*(12/hardwareMap.getAll(VoltageSensor.class).get(0).getVoltage()));
        rightFront.setPower(frontRightPower*(12/hardwareMap.getAll(VoltageSensor.class).get(0).getVoltage()));
        rightBack.setPower(backRightPower*(12/hardwareMap.getAll(VoltageSensor.class).get(0).getVoltage()));
    }

    double Clip(double Speed,double lim) {return Math.max(Math.min(Speed,lim),-lim);}

    public void whileInTimer(){
        robotCentricDrive(leftFront, leftBack, rightFront, rightBack, lim, gamepad1);
    }
}

