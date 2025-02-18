package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.actions.G2_Action;
import org.firstinspires.ftc.teamcode.actions.Score;
import org.firstinspires.ftc.teamcode.actions.Collect;
import org.firstinspires.ftc.teamcode.actions.Prepare;
import org.firstinspires.ftc.teamcode.actions.robot_drive;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.RobotMap;
import org.firstinspires.ftc.teamcode.systems.clawRotate_controller;
import org.firstinspires.ftc.teamcode.systems.claw_controller;
import org.firstinspires.ftc.teamcode.systems.linkage_controller;
import org.firstinspires.ftc.teamcode.systems.manualSlider_controller;
import org.firstinspires.ftc.teamcode.systems.sliderClaw_controller;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

@TeleOp(name = "TeleOp", group = "#")
public class Teleop extends LinearOpMode {
    RobotMap robot;
    slider_controller sliderController;
    claw_controller clawController;
    sliderClaw_controller sliderClawController;
    manualSlider_controller manualSliderController;
    clawRotate_controller clawRotateController;
    linkage_controller linkageController;
    Collect LinkageAction;
    Prepare SliderAction;
    Score scoreAction;
    robot_drive drive;
    G2_Action g2Action;
    int sliderPos;

    @Override
    public void runOpMode() {
        robot = new RobotMap(hardwareMap);

        sliderController         = new slider_controller(robot.slider);
        clawController           = new claw_controller(robot.claw);
        sliderClawController     = new sliderClaw_controller(robot.slider_claw);
        manualSliderController   = new manualSlider_controller(robot.slider);
        clawRotateController     = new clawRotate_controller(robot.claw_rotate);
        linkageController        = new linkage_controller(robot.linkage);

        SliderAction     = new Prepare(robot.slider_claw, robot.slider_claw_tilt, robot.turret, robot.slider, robot.claw, robot.leftFront, robot.leftBack, robot.rightFront, robot.rightBack, 1, robot.gamepad1, sliderPos);
        LinkageAction    = new Collect(robot.claw ,robot.claw_tilt, robot.linkage, robot.claw_rotate, robot.claw_pivot, robot.leftFront, robot.leftBack, robot.rightFront, robot.rightBack, 1, robot.gamepad1);
        scoreAction      = new Score(robot.claw, robot.claw_tilt, robot.linkage, robot.claw_rotate, robot.claw_pivot, robot.slider_claw, robot.slider_claw_tilt, robot.turret, robot.slider, LinkageAction, SliderAction, robot.leftFront, robot.leftBack, robot.rightFront, robot.rightBack, 1, robot.gamepad1);

        drive            = new robot_drive(robot.leftFront, robot.leftBack,robot.rightFront,robot.rightBack, 1,  robot.gamepad1);
        g2Action         = new G2_Action(robot.claw, robot.slider_claw, robot.claw_tilt, robot.claw_rotate, robot.claw_pivot, robot.linkage, robot.turret, robot.slider, robot.base_tilt, robot.slider_claw_tilt);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.robotCentricDrive(robot.leftFront, robot.leftBack, robot.rightFront, robot.rightBack, 1, gamepad1);

            if(gamepad1.dpad_right) {clawController.open_close();}

            if(gamepad1.options) {sliderClawController.open_close();}

            if(gamepad1.dpad_up) {clawRotateController.V_H();}

            if(gamepad1.dpad_down) {sliderController.setTargetPosition(Constants.SLIDER_DOWN);}

            if(gamepad1.right_bumper) {SliderAction.beforeTakeFromLinkage(); sliderController.setTargetPosition(SliderAction.SliderPos());}

            if(gamepad1.left_bumper){scoreAction.take();}

            if(gamepad1.a) {scoreAction.placeOnHighChamber();sliderController.setTargetPosition(SliderAction.SliderPos());}

            if(gamepad1.b) {scoreAction.placeInBasket();sliderController.setTargetPosition(SliderAction.SliderPos());}

            if(gamepad1.x) {SliderAction.takeFromHuman();sliderController.setTargetPosition(SliderAction.SliderPos());}

            if(gamepad1.y){SliderAction.takeFromLinkage();sliderController.setTargetPosition(SliderAction.SliderPos());}

            if(gamepad1.right_stick_button){scoreAction.score();sliderController.setTargetPosition(scoreAction.SliderPos());}

            if(gamepad2.right_stick_button && gamepad2.left_stick_button){g2Action.lev2Asent();}

            if(gamepad2.ps){g2Action.zeroPos(); sliderController.setTargetPosition(g2Action.SliderPos());}

            if(gamepad2.y){g2Action.switchBasketPos();}

            if(gamepad1.left_trigger>=0.01 || gamepad1.right_trigger>=0.01){manualSliderController.control(gamepad1.left_trigger, gamepad1.right_trigger); sliderController.setTargetPosition(manualSliderController.SliderPos());}
            clawRotateController.update();
            sliderClawController.update();
            sliderController.update();
            clawController.update();
            linkageController.update();

            telemetry.addData("slider:", sliderController.pos());
            telemetry.update();
        }
    }
}