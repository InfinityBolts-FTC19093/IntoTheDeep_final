package org.firstinspires.ftc.teamcode.utils.ToutchPad;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Pure Pursuit TeleOp")
public class TouchPad_TeleOp extends LinearOpMode {

    private PurePursuitController purePursuit;
    private List<Waypoint> path;
    private int i=2;

    private int multiplyer = 20;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        path = Arrays.asList(
                new Waypoint(gamepad1.touchpad_finger_1_x * multiplyer,gamepad1.touchpad_finger_1_y * multiplyer),
                new Waypoint(gamepad1.touchpad_finger_2_x * multiplyer, gamepad1.touchpad_finger_2_y * multiplyer)
        );
                purePursuit = new PurePursuitController(drive, path, 5.0);


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            path.add(i, new Waypoint(gamepad1.touchpad_finger_1_x * multiplyer, gamepad1.touchpad_finger_1_y * multiplyer));
            Timing.Timer timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);while (!timer.done()); timer.pause();
            i+=1;

            purePursuit.update();
            telemetry.update();
        }
    }
}
