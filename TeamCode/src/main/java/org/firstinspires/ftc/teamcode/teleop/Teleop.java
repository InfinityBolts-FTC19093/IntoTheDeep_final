package org.firstinspires.ftc.teamcode.teleop;
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
import org.firstinspires.ftc.teamcode.systems.sliderClaw_controller;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

@TeleOp(name = "TeleOp", group = "##")
public class Teleop extends LinearOpMode {
    RobotMap robot;
    slider_controller sliderController;
    claw_controller clawController;
    sliderClaw_controller sliderClawController;
    clawRotate_controller clawRotateController;
    linkage_controller linkageController;
    Collect LinkageAction;
    Prepare SliderAction;
    Score scoreAction;
    robot_drive drive;
    G2_Action g2Action;
    double lim = 1;

    @Override
    public void runOpMode() {
        robot = new RobotMap(hardwareMap);

        sliderController         = new slider_controller(robot.slider, hardwareMap);
        clawController           = new claw_controller(robot.claw);
        sliderClawController     = new sliderClaw_controller(robot.slider_claw);
        clawRotateController     = new clawRotate_controller(robot.claw_rotate);
        linkageController        = new linkage_controller(robot.linkage);

        SliderAction     = new Prepare(robot.slider_claw, robot.slider_claw_tilt, robot.turret, robot.slider, robot.claw, robot.leftFront, robot.leftBack, robot.rightFront, robot.rightBack, lim, gamepad1);
        LinkageAction    = new Collect(robot.claw ,robot.claw_tilt, robot.linkage, robot.claw_rotate, robot.claw_pivot, robot.colorRotateClaw, robot.colorCenterClaw, robot.leftFront, robot.leftBack, robot.rightFront, robot.rightBack, lim, gamepad1);
        scoreAction      = new Score(robot.claw, robot.claw_tilt, robot.linkage, robot.claw_rotate, robot.claw_pivot, robot.slider_claw, robot.slider_claw_tilt, robot.turret, robot.slider, LinkageAction, SliderAction, robot.leftFront, robot.leftBack, robot.rightFront, robot.rightBack, lim, gamepad1);
        drive            = new robot_drive(robot.leftFront, robot.leftBack,robot.rightFront,robot.rightBack, lim,  robot.gamepad1, hardwareMap);
        g2Action         = new G2_Action(robot.claw, robot.slider_claw, robot.claw_tilt, robot.claw_rotate, robot.claw_pivot, robot.linkage, robot.turret, robot.slider, robot.slider_claw_tilt);

        Prepare.setSliderController(sliderController);
        Score.setSliderController(sliderController);
        Collect.setLinkageController(linkageController);
        Collect.setDrive(drive);
        Prepare.setDrive(drive);
        Score.setDrive(drive);
        G2_Action.setSliderController(sliderController);
        G2_Action.setLinkageController(linkageController);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            drive.robotCentricDrive(robot.leftFront, robot.leftBack, robot.rightFront, robot.rightBack, lim, gamepad1);
            robot.claw_pivot.setPosition(0);

            if(gamepad1.options) {sliderClawController.open_close();}

            if(gamepad1.share){scoreAction.LinkagePlaceInSlider();}

            if(gamepad1.dpad_right) {clawController.open_close();}

            if(gamepad1.dpad_up) {clawRotateController.rotation();}

            if(gamepad1.dpad_down) {sliderController.setTargetPosition(Constants.SLIDER_DOWN);}

            if(gamepad1.right_bumper) {SliderAction.beforeTakeFromLinkage();}

            if(gamepad1.left_bumper){scoreAction.take();}

            if(gamepad1.a) {scoreAction.placeOnHighChamber();}

            if(gamepad1.b) {scoreAction.placeInBasket();}

            if(gamepad1.x) {SliderAction.takeFromHuman();}

            if(gamepad1.y){SliderAction.takeFromLinkage();}

            if(gamepad1.right_stick_button){scoreAction.score();}

            if(gamepad1.right_trigger >=.5){scoreAction.observation();}

            if(gamepad2.ps){g2Action.zeroPos();}

            if(gamepad2.y){g2Action.switchBasketPos();}

            sliderController.control(gamepad2.left_trigger, gamepad2.right_trigger);
            clawRotateController.update();
            sliderClawController.update();
            sliderController.update();
            clawController.update();
            linkageController.update();
        }
    }
}