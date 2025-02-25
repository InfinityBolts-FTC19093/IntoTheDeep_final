package org.firstinspires.ftc.teamcode.autonomous.actions;

import static org.firstinspires.ftc.teamcode.constants.Constants.WAIT_FOR_LINKAGE_ACTION;
import static org.firstinspires.ftc.teamcode.constants.Constants.WAIT_FOR_SLIDER_ACTION;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.systems.clawRotate_controller;
import org.firstinspires.ftc.teamcode.systems.claw_controller;
import org.firstinspires.ftc.teamcode.systems.linkage_controller;
import org.firstinspires.ftc.teamcode.systems.sliderClaw_controller;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

import java.util.concurrent.TimeUnit;

public class actions {
    public static class scoreAuto {
        CollectAuto LinkageAction;
        ScoreAuto scoreAction;
        Servo claw, claw_tilt, linkage, claw_rotate, claw_pivot, slider_claw, slider_claw_tilt, turret;
        DcMotorEx slider;
        slider_controller sliderController;
        claw_controller clawController;
        sliderClaw_controller sliderClawController;
        clawRotate_controller clawRotateController;
        linkage_controller linkageController;
        Timing.Timer timer;

        public scoreAuto(HardwareMap hardwareMap) {
            slider = hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_SLIDER);
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            claw = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW);
            slider_claw = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDER_CLAW);
            claw_rotate = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_ROTATE);
            claw_tilt = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_TILT);
            linkage = hardwareMap.get(Servo.class, HardwareConstants.ID_LINKAGE_SERVO);
            claw_pivot = hardwareMap.get(Servo.class, HardwareConstants.ID_PIVOT);
            turret = hardwareMap.get(Servo.class, HardwareConstants.ID_TURRET);
            slider_claw_tilt = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDER_CLAW_TILT);

            sliderController = new slider_controller(slider);
            clawController = new claw_controller(claw);
            sliderClawController = new sliderClaw_controller(slider_claw);
            clawRotateController = new clawRotate_controller(claw_rotate);
            linkageController = new linkage_controller(linkage);
        }

        public scoreAuto(CollectAuto LinkageAction, ScoreAuto ScoreAction) {
            this.scoreAction = ScoreAction;
            this.LinkageAction = LinkageAction;

        }

        public class UpdateAll implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                sliderController.update();
                clawController.update();
                clawRotateController.update();
                sliderClawController.update();
                linkageController.update();
                return true;
            }
        }
        public Action updateAll() {return new scoreAuto.UpdateAll();}

        public void initAll() {
            claw.setPosition(Constants.CLOSE_CLAW);
            slider_claw.setPosition(Constants.CLOSE_CLAW);
            claw_rotate.setPosition(Constants.ROTATE_INIT);
            claw_tilt.setPosition(Constants.TILT_INIT);
            linkage.setPosition(Constants.LINKAGE_INIT_POS);
            claw_pivot.setPosition(Constants.CLAW_ASSEMBLY_INIT);
            turret.setPosition(Constants.TURRET_INIT_AUTO);
            slider_claw_tilt.setPosition(Constants.SLIDER_TILT_INIT);
        }

        public class HighChamber implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                scoreAction.placeOnHighChamber();
                return false;
            }
        }

        public class HighBasket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
                    LinkageAction.placeInSlider();
                    timer = new Timing.Timer(WAIT_FOR_LINKAGE_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();
                }

                if(Constants.currentSliderActionPos == Constants.SliderActionPos.BEFORE_TAKE_FROM_LINKAGE){
                    if(Constants.currentClawPos == Constants.ClawPos.OPEN_CLAW){
                        claw.setPosition(Constants.CLOSE_CLAW);
                        Constants.currentClawPos = Constants.ClawPos.CLOSE_CLAW;
                        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();
                    }

                    if(Constants.currentSliderClawPos == Constants.SliderClawPos.CLOSE_CLAW){
                        slider_claw.setPosition(Constants.OPEN_CLAW);
                        Constants.currentSliderClawPos = Constants.SliderClawPos.OPEN_CLAW;
                        timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();
                    }

                    turret.setPosition(Constants.TURRET_TAKE_FROM_LINKAGE);
                    timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();

                    slider_claw_tilt.setPosition(Constants.SLIDER_TILT_TAKE_FROM_LINKAGE);
                    timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();

                    sliderController.setTargetPosition(Constants.SLIDER_TAKE_FORM_LINKAGE);
                    timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while(!timer.done()) {sliderController.update();} timer.pause();

                    slider_claw.setPosition(Constants.CLOSE_CLAW);
                    timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();

                    Constants.currentSliderActionPos = Constants.SliderActionPos.TAKE_FOR_LINKAGE;
                    timer = new Timing.Timer(WAIT_FOR_SLIDER_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();
                }

                claw.setPosition(Constants.OPEN_CLAW);
                Constants.currentClawPos = Constants.ClawPos.OPEN_CLAW;

                slider_claw.setPosition(Constants.CLOSE_CLAW);
                Constants.currentSliderClawPos = Constants.SliderClawPos.CLOSE_CLAW;
                timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done())timer.pause();

                sliderController.setTargetPosition(Constants.SLIDER_HIGH_BUSKET);
                timer = new Timing.Timer(200, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){sliderController.update();}timer.pause();

                slider_claw_tilt.setPosition(Constants.SLIDER_TILT_PLACE_IN_HIGH_BUSKET);
                turret.setPosition(Constants.TURRET_PLACE);

                Constants.currentSliderActionPos = Constants.SliderActionPos.PLACE_IN_BUSKET;

                Constants.currentScorePos = Constants.ScorePos.BUSKET;
                return false;
            }
        }

        public class BasketPreload implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                scoreAction.BasketPreload();
                return false;
            }
        }

        public class Place implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                scoreAction.score();
                return false;
            }
        }


//            public class TakeFromHuman implements Action {
//
//                @Override
//                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                    scoreAction.takeFromHuman();
//                    return false;
//                }
//            }

            public class TakeFromGround implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    scoreAction.take();
                    return false;
                }
            }

            public Action Chamber() {
                return new HighChamber();
            }

            public Action Basket() {
                return new HighBasket();
            }

        //    public Action TakeFromHuman() {return new TakeFromHuman();}

            public Action TakeFromGround() {
                return new TakeFromGround();
            }

            public Action Place() {
                return new Place();
            }

            public Action BasketPreload() {
                return new BasketPreload();
            }
        }
    }


