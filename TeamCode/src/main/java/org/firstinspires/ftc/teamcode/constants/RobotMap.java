package org.firstinspires.ftc.teamcode.constants;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotMap {

    public DcMotorEx leftFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightBack = null;
    public DcMotorEx slider = null;
    //public IMU imu = null;


    public Servo linkage;
    public Servo turret;
    public Servo claw_tilt;
    public Servo claw_rotate;
    public Servo claw;
    public Servo slider_claw;
    public Servo slider_claw_rotate;
    public Servo slider_claw_tilt;
    public Servo unghi_robot;

    public Gamepad gamepad1;

    public RobotMap(HardwareMap hardwareMap){
        leftFront  = hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_LEFT_FRONT);
        leftBack   = hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_LEFT_BACK);
        rightFront = hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_RIGHT_FRONT);
        rightBack  = hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_RIGHT_BACK);

        slider = hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_SLIDER);

        claw = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW);
        linkage = hardwareMap.get(Servo.class, HardwareConstants.ID_LINKAGE_SERVO);
        claw_rotate = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_ROTATE);
        claw_tilt = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_TILT);
        turret = hardwareMap.get(Servo.class, HardwareConstants.ID_ROTATE_CLAW_ASSEMBLY);

        slider_claw = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDER_CLAW);
        slider_claw_rotate = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDER_CLAW_ROTATE);
        slider_claw_tilt = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDER_CLAW_TILT);

        unghi_robot = hardwareMap.get(Servo.class, HardwareConstants.ID_MODIFICA_UNGHI_ROBOT);

        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

//        claw.setDirection(Servo.Direction.REVERSE);
//        linkage.setDirection(Servo.Direction.REVERSE);
//        claw_rotate.setDirection(Servo.Direction.REVERSE);
//        claw_titl.setDirection(Servo.Direction.REVERSE);
//        slider_claw.setDirection(Servo.Direction.REVERSE);
//        slider_claw_rotate.setDirection(Servo.Direction.REVERSE);
//        slider_claw_tilt.setDirection(Servo.Direction.REVERSE);
//        unghi_robot.setDirection(Servo.Direction.REVERSE);
//        rotate_claw_assembly.setDirection(Servo.Direction.REVERSE);


//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
//        );
//        imu.initialize(parameters);
        gamepad1 = new Gamepad();

    }
}
