package org.firstinspires.ftc.teamcode.systems.slider;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.Constants;

public class slider_controller {

    private final DcMotorEx slider;

    int targetPosition;
    double integral = 0;
    double previousError = 0;

    ElapsedTime timer;

    public slider_controller(DcMotorEx slider) {
        this.slider = slider;
        this.timer = new ElapsedTime();
        this.targetPosition = pos();
    }

    public void setTargetPosition(int targetPosition){
        this.targetPosition = targetPosition;
        this.integral = 0;
        this.previousError = 0;
    }

    public int pos(){
        return slider.getCurrentPosition();
    }

    public double getTargetPosition(){
        return targetPosition;
    }

    public void stopMotor(){
        slider.setPower(0);
    }

    public void update() {
        if (pos() <= 50) {
            stopMotor();
        } else {
            double currentPosition = slider.getCurrentPosition();
            double error = targetPosition - currentPosition;

            double proportional = Constants.Slider_kP * error;

            integral += error * timer.seconds();
            double integralTerm = Constants.Slider_kI * integral;

            double derivative = (error - previousError) / timer.seconds();
            double derivativeTerm = Constants.Slider_kD * derivative;

            double output = 0;
            output = proportional + integralTerm + derivativeTerm;
            output = Math.max(-1, Math.min(output, 1));

            slider.setPower(output);

            timer.reset();
            previousError = error;
        }
    }
}