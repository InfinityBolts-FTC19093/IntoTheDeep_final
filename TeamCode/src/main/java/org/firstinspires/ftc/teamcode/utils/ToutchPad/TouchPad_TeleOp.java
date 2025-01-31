package org.firstinspires.ftc.teamcode.utils.ToutchPad;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.constants.RobotMap;

import java.util.Vector;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "TouchPad-TeleOp")
public class TouchPad_TeleOp extends LinearOpMode {
    Timing.Timer timer;
    RobotMap robot = new RobotMap(hardwareMap);
    Pose2d startPos = new Pose2d(0, 0, 0);
    MecanumDrive drive = new MecanumDrive(hardwareMap, startPos);

    @Override
    public void runOpMode() {
        int vectorCapacity = 100;
        float multiplier = 41;

        Vector<Float> finger = new Vector<>(vectorCapacity);

        waitForStart();

        drive.resetOdometry();

        while(opModeIsActive() && !isStopRequested()){
            for(int i = 0; i< vectorCapacity; i+=2){
                finger.add(i,gamepad1.touchpad_finger_1_x* multiplier);
                finger.add(i+1, gamepad1.touchpad_finger_1_y* multiplier);
                timer = new Timing.Timer(100, TimeUnit.MILLISECONDS); while(!timer.done()){} timer.pause();
            }
            for(int i = 0; i< vectorCapacity; i+=2){
                drive.drive(finger.get(i), finger.get(i+1), 0);
                drive.updateOdometry();
            }
            drive.resetOdometry();
        }
    }
}
