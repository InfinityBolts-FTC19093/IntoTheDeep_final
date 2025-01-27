package org.firstinspires.ftc.teamcode.actions;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.Constants;

public class robot_drive {
    DcMotorEx leftFront, leftBack, rightFront, rightBack;
    double lim;
    IMU imu;

    //    double  PrecisionDenominator=1, PrecisionDenominator2=1;

    public robot_drive(DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, double lim , IMU imu){
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
        this.lim = lim;
        this.imu = imu;
    }

    public void robotCentricDrive() {
        Constants.currentRobotDriveStatus = Constants.RobotDriveStatus.ROBOT_CENTRIC;
        double y = -gamepad1.left_stick_y;
        double x =  gamepad1.left_stick_x* 1;
        double rx = gamepad1.right_stick_x*1;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

//        rx/=PrecisionDenominator2;
//        x/=PrecisionDenominator;
//        y/=PrecisionDenominator;

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftPower = Clip(frontLeftPower,lim);
        backLeftPower = Clip(backLeftPower,lim);
        frontRightPower = Clip(frontRightPower,lim);
        backRightPower = Clip(backRightPower,lim);

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }

    double Clip(double Speed,double lim) {return Math.max(Math.min(Speed,lim),-lim);}

    public void FieldCentricDrive(){
        Constants.currentRobotDriveStatus = Constants.RobotDriveStatus.FIELD_CENTRIC;
        // Adjust the orientation parameters to match your robot

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options && gamepad2.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }

    public void switchDrive(){
        if(Constants.currentRobotDriveStatus == Constants.RobotDriveStatus.ROBOT_CENTRIC){
            FieldCentricDrive();
        }else if (Constants.currentRobotDriveStatus == Constants.RobotDriveStatus.FIELD_CENTRIC){
            robotCentricDrive();
        }
    }
}

