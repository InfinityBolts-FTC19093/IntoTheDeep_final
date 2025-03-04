package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.constants.Constants;

@Autonomous (name = "Reset Pinpoint", group = "Reset")
public class ResetPinpoint extends LinearOpMode {
    GoBildaPinpointDriverRR odo;

    @Override
    public void runOpMode() throws InterruptedException {
        odo = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");
        odo.setOffsets(Constants.X_OFFSET, Constants.Y_OFFSET);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
        telemetry.addData("Status", odo.getDeviceStatus());
        telemetry.addData("X Encoder:", odo.getEncoderX());
        telemetry.addData("Y Encoder:", odo.getEncoderY());
        telemetry.update();
        waitForStart();

    }
}
