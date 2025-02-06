package org.firstinspires.ftc.teamcode.utils.ToutchPad;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MecanumDrive {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private BNO055IMU imu;

    public MecanumDrive(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "lf");
        frontRight = hardwareMap.get(DcMotorEx.class, "rf");
        backLeft = hardwareMap.get(DcMotorEx.class, "lb");
        backRight = hardwareMap.get(DcMotorEx.class, "rb");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
    }

    public Pose2d getPoseEstimate() {
        Orientation angles = imu.getAngularOrientation();
        return new Pose2d(0, 0, Math.toRadians(angles.firstAngle)); // Replace with localization
    }

    public void setDrivePowers(double speed, double headingError) {
        double turn = headingError * 0.5;
        frontLeft.setPower(speed + turn);
        frontRight.setPower(speed - turn);
        backLeft.setPower(speed + turn);
        backRight.setPower(speed - turn);
    }
}
