package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.actions.robot_drive;
import org.firstinspires.ftc.teamcode.constants.RobotMap;


@TeleOp(name = "TeleOp-sasiu", group = "#")
public class TeleOp_sasiu extends LinearOpMode {
    RobotMap robot;
    robot_drive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotMap(hardwareMap);
        drive = new robot_drive(robot.leftFront, robot.leftBack,robot.rightFront,robot.rightBack, 1,  robot.gamepad1, hardwareMap);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            //drive.robotCentricDrive(robot.leftFront, robot.leftBack, robot.rightFront, robot.rightBack, 1, gamepad1);
            robot.turret.setPosition(0.025);
            robot.claw.setPosition(0);
            robot.slider_claw_tilt.setPosition(0);
            robot.slider_claw.setPosition(0);
            robot.claw_rotate.setPosition(0);
            robot.linkage.setPosition(0);
            robot.claw_pivot.setPosition(0);
            robot.claw_tilt.setPosition(0);
        }
    }
}
