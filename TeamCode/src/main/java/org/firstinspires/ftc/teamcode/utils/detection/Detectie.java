package org.firstinspires.ftc.teamcode.utils.detection;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Detectie {
    Limelight3A limelight;

    public Detectie(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public void start() {
        limelight.start();
    }

    public void stop () {
        limelight.stop();
    }

    public double[] getLimelightData() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            double[] pythonOutput = result.getPythonOutput();
            if (pythonOutput != null && pythonOutput.length >= 4) {
                return pythonOutput;
            }
        }
        return null; // Return null if there is no data available
    }
}
