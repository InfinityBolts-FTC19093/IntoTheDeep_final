package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.constants.Constants.WAIT_FOR_LINKAGE_ACTION;
import static org.firstinspires.ftc.teamcode.constants.Constants.WAIT_FOR_SLIDER_ACTION;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.actions.Collect;
import org.firstinspires.ftc.teamcode.actions.Prepare;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.systems.autoClaw_controller;
import org.firstinspires.ftc.teamcode.systems.sliderClaw_controller;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

import java.util.concurrent.TimeUnit;

public class actions {
        public static class Lift {
            private final DcMotorEx slider;
            private final slider_controller sliderController;

            public Lift(HardwareMap hardwareMap) {
                slider = hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_SLIDER);
                slider.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                slider.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                slider.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

                sliderController = new slider_controller(slider);
            }

            public class Update implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    sliderController.update();
                    telemetryPacket.put("pos", sliderController.pos());
                    return true;
                }
            }

            public class MoveSlider implements Action {
                private final int targetPosition;

                public MoveSlider (int targetPosition) {this.targetPosition = targetPosition;}
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    sliderController.setTargetPosition(targetPosition);
                    telemetryPacket.put("target pos", targetPosition);
                    return false;
                }
            }

            public Action Update() {return new Update();}
        }

    public static class Claw {
        private Servo claw;
        public Claw (HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW);
            //  claw.setDirection(Servo.Direction.REVERSE);
        }
        public class Open implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(Constants.OPEN_CLAW);
                return false;
            }
        }
        public Action open()    {return new Open();}
    }

        public static class Score {
            Timing.Timer timer;
            Servo claw, claw_tilt, linkage, claw_rotate, rotate_claw_assembly, slider_claw, slider_claw_tilt, slider_claw_rotate;
            DcMotorEx slider;
            Collect LinkageAction;
            Prepare SliderAction;
            sliderClaw_controller sliderClawController;
            slider_controller sliderController;

            public Score(HardwareMap hardwareMap) {
                Collect LinkageAction = new Collect(claw ,claw_tilt, linkage, claw_rotate, rotate_claw_assembly);
                Prepare SliderAction  = new Prepare(slider_claw, slider_claw_tilt, slider_claw_rotate, slider);

                sliderClaw_controller sliderClawController = new sliderClaw_controller(slider_claw);
                slider_controller sliderController         = new slider_controller(slider);
            }

            autoClaw_controller autoClawController = new autoClaw_controller();

            public class HighChamber implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
                        LinkageAction.placeInSlider();
                        timer = new Timing.Timer(WAIT_FOR_LINKAGE_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
                    }

                    if(Constants.currentSliderActionPos == Constants.SliderActionPos.PLACE_ON_CHAMBER || Constants.currentSliderActionPos == Constants.SliderActionPos.INIT){
                        SliderAction.takeFromLinkage();
                        timer = new Timing.Timer(WAIT_FOR_SLIDER_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
                    }

                    SliderAction.placeOnHighChamber();
                    timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

                    Constants.currentScorePos = Constants.ScorePos.CHAMBER;
                    return false;
                }
            }

            public class HighBasket implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    if(Constants.currentLinkageActionPos == Constants.LinkageActionPos.TAKE || Constants.currentLinkageActionPos == Constants.LinkageActionPos.INIT){
                        LinkageAction.placeInSlider();
                        timer = new Timing.Timer(WAIT_FOR_LINKAGE_ACTION, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
                    }
                    SliderAction.placeOnHighBusket();

                    Constants.currentScorePos = Constants.ScorePos.BUSKET;
                    return false;
                }
            }

            public class PlaceSample implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    if(Constants.currentScorePos == Constants.ScorePos.CHAMBER){
                        sliderController.setTargetPosition(Constants.SLIDER_HIGH_CHAMBER-200);
                        timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while (!timer.done()){sliderController.update();}timer.pause();
                        sliderClawController.setPos(Constants.OPEN_CLAW);
                }
                    return false;
            }
        }

        public class TakeFromHuman implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(Constants.currentSliderClawPos == Constants.SliderClawPos.CLOSE_CLAW){
                    claw.setPosition(Constants.SLIDER_OPEN);
                    timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();
                }

                claw_rotate.setPosition(Constants.SLIDER_ROTATE_TAKE_HUMAN);
                slider_claw_tilt.setPosition(Constants.SLIDER_TILT_TAKE_FORM_HUMAN);

                sliderController.setTargetPosition(Constants.SLIDER_DOWN);
                return false;
            }
        }

        public class TakeFromGround implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                linkage.setPosition(Constants.LINKAGE_TAKE_POS);

                claw.setPosition(Constants.OPEN_CLAW);
                rotate_claw_assembly.setPosition(Constants.CLAW_ASSEMBLY_TAKE);
                claw_tilt.setPosition(Constants.TILT_TAKE);

                timer = new Timing.Timer(50, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

                autoClawController.take();

                timer = new Timing.Timer(150, TimeUnit.MILLISECONDS);timer.start();while (!timer.done());timer.pause();

                Constants.currentLinkageActionPos = Constants.LinkageActionPos.TAKE;
                return false;
            }
        }

        public Action Chamber()         {return new HighChamber();}
        public Action Basket()          {return new HighBasket(); }
        public Action TakeFromHuman()   {return new TakeFromHuman();}
        public Action TakeFromGround()  {return new TakeFromGround();}
        public Action PlaceSample()     {return new PlaceSample();}
    }
}
