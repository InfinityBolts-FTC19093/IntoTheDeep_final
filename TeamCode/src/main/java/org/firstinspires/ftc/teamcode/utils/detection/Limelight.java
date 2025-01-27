package org.firstinspires.ftc.teamcode.utils.detection;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "ZLimelight Servo Control", group = "Test")
public class Limelight extends LinearOpMode {

    private Limelight3A limelight;
    private Servo servo;
    private static final double CENTER_POSITION = 0.5;
    private static final double ADJUSTMENT_SCALE = 0.005;

    @Override
    public void runOpMode()  {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        servo = hardwareMap.get(Servo.class, "servo");

        servo.setPosition(CENTER_POSITION);

        limelight.start();

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // Get horizontal offset (tx) from Limelight
                double tx = result.getTx();

                // Calculate new servo position
                double newPosition = CENTER_POSITION - (tx * ADJUSTMENT_SCALE);

                // Clamp servo position between 0.0 and 1.0
                newPosition = Math.max(0.0, Math.min(1.0, newPosition));

                // Set servo position
                servo.setPosition(newPosition);

                // Display telemetry
                telemetry.addData("Limelight tx", tx);
                telemetry.addData("Servo Position", newPosition);
            } else {
                telemetry.addData("Limelight", "No valid target");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
