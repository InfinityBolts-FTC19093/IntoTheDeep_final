package org.firstinspires.ftc.teamcode.utils.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.RobotMap;

public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotMap robot = new RobotMap(hardwareMap);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            if(gamepad1.a){
                robot.claw.setPosition(Constants.CLOSE_CLAW);
            }
            if(gamepad1.b){
                robot.claw.setPosition(Constants.OPEN_CLAW);
            }
        }
    }
}
