package org.firstinspires.ftc.teamcode.utils.tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.HardwareConstants;


@Config
@Autonomous(name = "PIDTest", group = "Test")
public class PIDtest extends LinearOpMode {
    DcMotorEx slider;

    private double integralSum = 0;

    public static double Kp = 0.01;
    public static double Ki = 0;
    public static double Kd = 0;

    public static double distance = 120;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);


        slider = hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_SLIDER);

        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        packet.put("Power", 0.0);
        packet.put("Position", slider.getCurrentPosition());
        packet.put("error", lastError);

        //sleep(300);

        while(opModeIsActive() && !isStopRequested()) {
            double power = PIDControl(distance, slider.getCurrentPosition());

            packet.put("Power_slider", power);
            packet.put("Position_slider", slider.getCurrentPosition());
            packet.put("error_slider", lastError);

            slider.setPower(power);

            dashboard.sendTelemetryPacket(packet);
        }
    }

    public double PIDControl(double reference, double state) {
        double error = reference - state;

        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error*Kp) + (derivative*Kd) + (integralSum*Ki);
        return output;
    }
}
