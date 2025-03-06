package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.constants.Constants;

public class slider_controller {

    private final DcMotorEx slider;
    private double targetPosition;
    double integral = 0;
    double previousError = 0;
    int pos;
    HardwareMap hardwareMap;

    ElapsedTime timer;

    public slider_controller(DcMotorEx slider, HardwareMap hardwareMap) {
        this.slider = slider;
        this.timer = new ElapsedTime();
        this.targetPosition = pos();
        this.hardwareMap = hardwareMap;
    }

    public void setTargetPosition(double targetPosition){
        this.targetPosition = targetPosition;
        this.integral = 0;
        this.previousError = 0;
    }

    public double pos(){
        return slider.getCurrentPosition();
    }

    public double getTargetPosition(){
        return targetPosition;
    }

    public void stopMotor(){
        slider.setPower(0);
    }

    public void update() {
        if (pos() <= 25 && targetPosition <= 25) {
            stopMotor();
        } else {
            double currentPosition = slider.getCurrentPosition();
            double error = targetPosition - currentPosition;

            double proportional = Constants.Slider_kPAuto * error;

            integral += error * timer.seconds();
            double integralTerm = Constants.Slider_kI * integral;

            double derivative = (error - previousError) / timer.seconds();
            double derivativeTerm = Constants.Slider_kD * derivative;

            double output = 0;
            output = proportional + integralTerm + derivativeTerm;
            output = Math.max(-1, Math.min(output, 1));

            slider.setPower(output*(12/hardwareMap.getAll(VoltageSensor.class).get(0).getVoltage()));

            timer.reset();
            previousError = error;
        }
    }

    public void control(double leftTrigger, double rightTrigger){
        if (leftTrigger>= 0.05) {
            slider.setPower(-leftTrigger);
            pos = slider.getCurrentPosition();
            setTargetPosition(pos);
            if (pos() <= 0) {
                setTargetPosition(Constants.SLIDER_DOWN);
            }
        }
        if (rightTrigger >= 0.05) {
            slider.setPower(rightTrigger);
            pos = slider.getCurrentPosition();
            setTargetPosition(pos);
        }
    }
}