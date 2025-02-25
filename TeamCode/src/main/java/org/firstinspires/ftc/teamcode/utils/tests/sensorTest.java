package org.firstinspires.ftc.teamcode.utils.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;


@TeleOp(name = "test", group = "#")
public class sensorTest extends LinearOpMode {
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "color");

        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        waitForStart();
        while(opModeIsActive() && !isStopRequested()){

            telemetry.addData("RED", colorSensor.red());
            telemetry.addData("BLUE", colorSensor.blue());
            telemetry.addData("GREEN", colorSensor.green());
            telemetry.addData("ALPHA", colorSensor.alpha());
            telemetry.update();
        }
    }
}
