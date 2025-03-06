package org.firstinspires.ftc.teamcode.utils.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp (name = "servo Test", group = "tests")
public class servoTest extends OpMode {
    Servo servo;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");

    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            servo.setPosition(1);
        }
        else {
            servo.setPosition(0);
        }
    }
}
