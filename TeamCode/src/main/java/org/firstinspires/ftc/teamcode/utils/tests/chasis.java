package org.firstinspires.ftc.teamcode.utils.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.RobotMap;

@TeleOp (name = "Chasis", group = "Test")
public class chasis extends OpMode {
    private RobotMap r;
    @Override
    public void init() {
        r = new RobotMap(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            r.leftBack.setPower(1);
            telemetry.addData("a", "leftBack");
            telemetry.update();
        }
        else if (gamepad1.b) {
            r.leftFront.setPower(1);
            telemetry.addData("b", "leftFront");
            telemetry.update();
        }
        else if (gamepad1.y) {
            r.rightFront.setPower(1);
            telemetry.addData("y", "rightFront");
            telemetry.update();
        }
        else if (gamepad1.x) {
            r.rightBack.setPower(1);
            telemetry.addData("x", "rightBack");
            telemetry.update();
        }
        else {
            r.rightBack.setPower(0);
            r.rightFront.setPower(0);
            r.leftFront.setPower(0);
            r.leftBack.setPower(0);
        }

        // lb -> lf
        //lf -> rf
        //rf -> lb
        //
    }
}
