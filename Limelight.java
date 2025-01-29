package org.firstinspires.ftc.teamcode.detection;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Limelight Orientation Servo", group = "Robot")
public class Limelight extends LinearOpMode {

    private Servo sampleServo;
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // Initialize servo
        sampleServo = hardwareMap.get(Servo.class, "sampleServo");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();

        // Wait for the start of the OpMode
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            double[] inputs = {1.0, 2.0, 3.0};

            // Send inputs to the Python SnapScript
            limelight.updatePythonInputs(inputs);

            // Retrieve outputs from the Python SnapScript
            double[] pythonOutputs = result.getPythonOutput();

            if (pythonOutputs != null && pythonOutputs.length > 0) {
                // Assuming orientation is encoded in the first Python output
                double orientation = pythonOutputs[0];

                // Control the servo based on the orientation
                if (orientation == 1.0) { // Horizontal
                    telemetry.addData("Orientation", "Horizontal");
                    sampleServo.setPosition(0.2); // Example: Move servo to the left
                } else if (orientation == 2.0) { // Vertical
                    telemetry.addData("Orientation", "Vertical");
                    sampleServo.setPosition(0.8); // Example: Move servo to the right
                } else {
                    telemetry.addData("Orientation", "None");
                    sampleServo.setPosition(0.5); // Neutral position
                }

                // Display telemetry for debugging
                telemetry.addData("Python Output[0]:", orientation);
            } else {
                telemetry.addData("Error", "No output from Python SnapScript");
                sampleServo.setPosition(0.5); // Neutral position if no data
            }

            telemetry.update();
        }
    }

}
