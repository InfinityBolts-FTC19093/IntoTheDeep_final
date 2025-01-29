package org.firstinspires.ftc.teamcode.constants;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotMap {

    public DcMotorEx leftFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightBack = null;
    public DcMotorEx slider = null;
    public IMU imu = null;

    public Servo claw;
    public Servo linkage;
    public Servo claw_rotate;
    public Servo claw_tilt;
    public Servo angle;

    public Servo liftRobot;

    public Servo slider_claw;
    public Servo slider_claw_rotate;
    public Servo slider_claw_tilt;

    public RobotMap(HardwareMap hardwareMap){
        leftFront  = hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_LEFT_FRONT);
        leftBack   = hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_LEFT_BACK);
        rightFront = hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_RIGHT_FRONT);
        rightBack  = hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_RIGHT_BACK);

   //     slider = hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_SLIDER);

        claw = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW);
        linkage = hardwareMap.get(Servo.class, HardwareConstants.ID_LINKAGE_SERVO);
        claw_rotate = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_ROTATE);
        claw_tilt = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_TILT);

        liftRobot= hardwareMap.get(Servo.class, HardwareConstants.ID_LIFT_ROBOT);

//        slider_claw = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDER_CLAW);
//        slider_claw_rotate = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDER_CLAW_ROTATE);
//        slider_claw_tilt = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDER_CLAW_TILT);

        angle = hardwareMap.get(Servo.class, HardwareConstants.ID_ANDGLE);

//        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

//        claw.setDirection(Servo.Direction.REVERSE);
//        linkage.setDirection(Servo.Direction.REVERSE);
//        claw_rotate.setDirection(Servo.Direction.REVERSE);
//        claw_titl.setDirection(Servo.Direction.REVERSE);
//        slider_claw.setDirection(Servo.Direction.REVERSE);
//        slider_claw_rotate.setDirection(Servo.Direction.REVERSE);
//        slider_claw_tilt.setDirection(Servo.Direction.REVERSE);
//        unghi_robot.setDirection(Servo.Direction.REVERSE);


//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
//        );
//        imu.initialize(parameters);

    }

    public void reset(){
        claw.setPosition(Constants.OPEN_CLAW);
        slider_claw.setPosition(Constants.SLIDER_OPEN);
        linkage.setPosition(Constants.LINKAGE_INIT_POS);
        claw_tilt.setPosition(Constants.TILT_INIT);

    }
}
