package org.firstinspires.ftc.teamcode.utils.detection;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "DetectieTest", group = "#")
public class testDetectie extends OpMode {
    Detectie detectie;
    @Override
    public void init() {
        detectie = new Detectie(hardwareMap);
        telemetry.setMsTransmissionInterval(11);
        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();

    }

    @Override
    public void loop() {
        double[] limelightData = detectie.getLimelightData();
        if (limelightData != null) {
            telemetry.addData("Cadranu ", limelightData[0]);
        }
        else {
            telemetry.addData("nu i", "pula mea");
        }
        telemetry.update();
    }
}
