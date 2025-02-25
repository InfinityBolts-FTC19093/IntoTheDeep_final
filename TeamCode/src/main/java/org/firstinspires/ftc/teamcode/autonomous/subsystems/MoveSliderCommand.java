package org.firstinspires.ftc.teamcode.autonomous.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.autonomous.subsystems.SliderSubsystem;

public class MoveSliderCommand extends CommandBase {
    private final SliderSubsystem sliderSubsystem;
    private final double targetPosition;

    public MoveSliderCommand(SliderSubsystem sliderSubsystem, double targetPosition) {
        this.sliderSubsystem = sliderSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(sliderSubsystem);
    }

    @Override
    public void initialize() {
        sliderSubsystem.setTargetPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(sliderSubsystem.getCurrentPosition() - targetPosition) < 5;
    }
}
