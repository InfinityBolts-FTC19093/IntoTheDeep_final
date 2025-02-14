package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.constants.RobotMap;

@Config
@TeleOp(name="ServoPos")
public class ServoPos extends OpMode {

    public static double ClawPos=0,LinkagePos=0,RotateClawPos=0,ClawTiltPos=0,SliderClawPos=0,SliderClawRotatePos=0,SliderClawTiltPos=0,ModificaUnghiPos=0,RotateAssemplyPos=0;
    private Servo claw, linkage, claw_rotate, claw_tilt, turret, slider_claw, slider_claw_rotate, slider_claw_tilt, unghi_robot;
    private DcMotorEx slider;
    @Override
    public void init() {
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
    }
}
