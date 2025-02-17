package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.constants.Constants;

public class manualSlider_controller {
    private DcMotorEx slider;
    private int SliderPos=0;
    private slider_controller sliderController;

    public manualSlider_controller(DcMotorEx slider){
        this.slider = slider;
        this.sliderController = new slider_controller(this.slider);
    }


    public void control(double leftTrigger, double rightTrigger){
        if (leftTrigger>= 0.01) {
            slider.setPower(-leftTrigger);
            SliderPos = slider.getCurrentPosition();
            sliderController.setTargetPosition(SliderPos);
            if (sliderController.pos() <= 0) {
                sliderController.setTargetPosition(Constants.SLIDER_DOWN);
            }
            sliderController.update();
        }
        if (rightTrigger >= 0.01) {
            slider.setPower(rightTrigger);
            SliderPos = slider.getCurrentPosition();
            sliderController.setTargetPosition(SliderPos);
            sliderController.update();
        }
    }
}
