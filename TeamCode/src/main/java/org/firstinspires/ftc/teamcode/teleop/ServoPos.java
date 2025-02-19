package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.teamcode.constants.HardwareConstants;

@Config
@TeleOp(name="ServoPos")
public class ServoPos extends OpMode {

    public static double ClawPos=0,LinkagePos=0,RotateClawPos=0,ClawTiltPos=0,SliderClawPos=0,SliderClawRotatePos=.5,SliderClawTiltPos=0,ModificaUnghiPos=0,RotateAssemplyPos=0;
    private Servo claw, linkage, claw_rotate, claw_tilt, turret, slider_claw, slider_claw_rotate, slider_claw_tilt, unghi_robot;
    private DcMotorEx slider;

    private FtcDashboard dashboard;

    @Override
    public void init() {
        slider = hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_SLIDER);

        claw = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW);
        linkage = hardwareMap.get(Servo.class, HardwareConstants.ID_LINKAGE_SERVO);
        claw_rotate = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_ROTATE);
        claw_tilt = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_TILT);
        turret = hardwareMap.get(Servo.class, HardwareConstants.ID_PIVOT);

        slider_claw = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDER_CLAW);
        slider_claw_rotate = hardwareMap.get(Servo.class, HardwareConstants.ID_TURRET);
        slider_claw_tilt = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDER_CLAW_TILT);

        unghi_robot = hardwareMap.get(Servo.class, HardwareConstants.ID_MODIFICA_UNGHI_ROBOT);

        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        telemetry.addData("Dashboard", "Connected");
        telemetry.update();
    }

    @Override
    public void loop() {
        claw.setPosition(ClawPos);
        linkage.setPosition(LinkagePos);
        claw_rotate.setPosition(RotateClawPos);
        claw_tilt.setPosition(ClawTiltPos);
        slider_claw.setPosition(SliderClawPos);
        slider_claw_rotate.setPosition(SliderClawRotatePos);
        slider_claw_tilt.setPosition(SliderClawTiltPos);
        unghi_robot.setPosition(ModificaUnghiPos);
        turret.setPosition(RotateAssemplyPos);

        telemetry.addData("slider:", slider.getCurrentPosition());

        telemetry.addData("Loop", "Running");
        telemetry.update();
    }
}
