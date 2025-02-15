package org.firstinspires.ftc.teamcode.utils.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.RobotMap;

@TeleOp(name = "odoTest", group = "Test")
public class odoTest extends OpMode {

    private RobotMap robot;

    @Override
    public void init() {robot = new RobotMap(hardwareMap);}

    @Override
    public void loop() {
        telemetry.addData("odo DR: ", robot.leftFront.getCurrentPosition());
        telemetry.addData("odo ST: ", robot.rightFront.getCurrentPosition());
        telemetry.addData("odo MIJ: ", robot.leftBack.getCurrentPosition());
        telemetry.update();
    }
}
