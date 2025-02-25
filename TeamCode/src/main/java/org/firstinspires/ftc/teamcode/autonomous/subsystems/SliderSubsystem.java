package org.firstinspires.ftc.teamcode.autonomous.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

public class SliderSubsystem extends SubsystemBase {
    private final slider_controller sliderController;
    private double lastTargetPosition;  // ✅ Holds the last target position

    public SliderSubsystem(DcMotorEx sliderMotor) {
        this.sliderController = new slider_controller(sliderMotor);
        this.lastTargetPosition = sliderController.pos(); // Start at current position
    }

    public void setTargetPosition(double target) {
        lastTargetPosition = target;  // ✅ Store new target
        sliderController.setTargetPosition(target);
    }

    public double getCurrentPosition() {
        return sliderController.pos();
    }

    @Override
    public void periodic() {
        // ✅ PID continues running to hold the last commanded position
        sliderController.update();
    }
}
