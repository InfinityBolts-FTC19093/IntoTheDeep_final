package org.firstinspires.ftc.teamcode.autonomous.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

public class SliderSubsystem extends SubsystemBase {
    private final slider_controller sliderController;
    private double lastTargetPosition;

    public SliderSubsystem(DcMotorEx sliderMotor) {
        this.sliderController = new slider_controller(sliderMotor);
        this.lastTargetPosition = sliderController.pos(); // Start at current position
    }

    public void setTargetPosition(double target) {
        lastTargetPosition = target;
        sliderController.setTargetPosition(target);
    }

    public double getCurrentPosition() {
        return sliderController.pos();
    }

    @Override
    public void periodic() {
        sliderController.update();
    }
}
