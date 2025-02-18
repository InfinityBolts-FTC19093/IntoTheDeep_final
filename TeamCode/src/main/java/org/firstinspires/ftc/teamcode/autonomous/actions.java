package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.actions.Collect;
import org.firstinspires.ftc.teamcode.actions.Prepare;
import org.firstinspires.ftc.teamcode.actions.Score;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.systems.clawRotate_controller;
import org.firstinspires.ftc.teamcode.systems.claw_controller;
import org.firstinspires.ftc.teamcode.systems.manualSlider_controller;
import org.firstinspires.ftc.teamcode.systems.sliderClaw_controller;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

import java.util.concurrent.TimeUnit;

public class actions {


    public static class Lift {
        private final DcMotorEx slider;
        private final slider_controller sliderController;

        public Lift(HardwareMap hardwareMap) {
            slider = hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_SLIDER);
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            sliderController = new slider_controller(slider);
        }
        public class Update implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                sliderController.update();
                packet.put("pos", sliderController.pos());
                return true;
            }
        }
        public class MoveSlides implements Action {
            private final int targetPosition;

            public MoveSlides(int targetPosition) {
                this.targetPosition = targetPosition;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                sliderController.setTargetPosition(targetPosition);
                packet.put("target pos", targetPosition);
                return false;
            }
        }

        public Action update() {return new Update();}
    }
    public static class Update {
        private final Servo claw, slider_claw, claw_rotate;
        private final DcMotorEx slider;
        private final slider_controller sliderController;
        private final claw_controller clawController;
        private final sliderClaw_controller sliderClawController;
        private final clawRotate_controller clawRotateController;

        public Update (HardwareMap hardwareMap) {
            slider = hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_SLIDER);
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            claw = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW);
            slider_claw = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDER_CLAW);
            claw_rotate = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_ROTATE);

            sliderController         = new slider_controller(slider);
            clawController           = new claw_controller(claw);
            sliderClawController     = new sliderClaw_controller(slider_claw);
            clawRotateController     = new clawRotate_controller(claw_rotate);
        }

        public class UpdateAll implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                sliderController.update();
                clawController.update();
                clawRotateController.update();
                sliderClawController.update();
                return true;
            }
        }
        public Action updateAll() {return new UpdateAll();}
    }


    public static class scoreAuto {
        Collect linkageAction;
        Prepare sliderAction;
        Score scoreAction;

        public scoreAuto(Score scoreAction, Prepare sliderAction, Collect linkageAction) {
            this.scoreAction = scoreAction;
            this.sliderAction = sliderAction;
            this.linkageAction = linkageAction;
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
                    scoreAction.placeInHighBasket();
                    return false;
                }
            }

            public class PlaceSample implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    scoreAction.placeSample();
                    return false;
                }
            }

            public class TakeFromHuman implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    sliderAction.takeFromHuman();
                    return false;
                }
            }

            public class TakeFromGround implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    scoreAction.take();
                    return false;
                }
            }

            public Action Chamber() {return new HighChamber();}

            public Action Basket() {
                return new HighBasket();
            }

            public Action TakeFromHuman() {
                return new TakeFromHuman();
            }

            public Action TakeFromGround() {
                return new TakeFromGround();
            }

            public Action PlaceSample() {
                return new PlaceSample();
            }
        }
    }

