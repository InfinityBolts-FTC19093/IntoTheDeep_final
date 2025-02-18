package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.constants.Constants.WAIT_FOR_LINKAGE_ACTION;
import static org.firstinspires.ftc.teamcode.constants.Constants.WAIT_FOR_SLIDER_ACTION;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.actions.Collect;
import org.firstinspires.ftc.teamcode.actions.InTimer;
import org.firstinspires.ftc.teamcode.actions.Prepare;
import org.firstinspires.ftc.teamcode.actions.Score;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.systems.clawRotate_controller;
import org.firstinspires.ftc.teamcode.systems.claw_controller;
import org.firstinspires.ftc.teamcode.systems.linkage_controller;
import org.firstinspires.ftc.teamcode.systems.sliderClaw_controller;
import org.firstinspires.ftc.teamcode.systems.slider_controller;

import java.util.concurrent.TimeUnit;

public class actions {
    public static class Update {
        private final Servo claw, claw_tilt, linkage, claw_rotate, claw_pivot, slider_claw, slider_claw_tilt, turret;
        private final DcMotorEx slider, leftFront, leftBack, rightFront, rightBack;
        private final slider_controller sliderController;
        private final claw_controller clawController;
        private final sliderClaw_controller sliderClawController;
        private final clawRotate_controller clawRotateController;
        private final linkage_controller linkageController;
        Score score;
        Prepare prepare;
        Collect collect;
        Gamepad gamepad1;
        int sliderPos;

        public Update(HardwareMap hardwareMap) {
            slider = hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_SLIDER);
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftFront = hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_LEFT_FRONT);
            leftBack = hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_LEFT_BACK);
            rightFront = hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_RIGHT_FRONT);
            rightBack = hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_RIGHT_BACK);

            claw = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW);
            slider_claw = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDER_CLAW);
            claw_rotate = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_ROTATE);
            claw_tilt = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_TILT);
            linkage = hardwareMap.get(Servo.class, HardwareConstants.ID_LINKAGE_SERVO);
            claw_pivot = hardwareMap.get(Servo.class, HardwareConstants.ID_PIVOT);
            turret = hardwareMap.get(Servo.class, HardwareConstants.ID_TURRET);
            slider_claw_tilt = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDER_CLAW_TILT);


            prepare = new Prepare(slider_claw, slider_claw_tilt, turret, slider, claw, leftFront, leftBack, rightFront, rightBack, 1, gamepad1, sliderPos);
            collect = new Collect(claw, claw_tilt, linkage, claw_rotate, claw_pivot, leftFront, leftBack, rightFront, rightBack, 1, gamepad1);
            score = new Score(claw, claw_tilt, linkage, claw_rotate, claw_pivot, slider_claw, slider_claw_tilt, turret, slider, collect, prepare, leftFront, leftBack, rightFront, rightBack, 1, gamepad1);

            sliderController = new slider_controller(slider);
            clawController = new claw_controller(claw);
            sliderClawController = new sliderClaw_controller(slider_claw);
            clawRotateController = new clawRotate_controller(claw_rotate);
            linkageController = new linkage_controller(linkage);

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

        public Action updateAll() {
            return new UpdateAll();
        }

        public void initAll() {
            claw.setPosition(Constants.CLOSE_CLAW);
            slider_claw.setPosition(Constants.CLOSE_CLAW);
            claw_rotate.setPosition(Constants.ROTATE_INIT);
            claw_tilt.setPosition(Constants.TILT_INIT);
            linkage.setPosition(Constants.LINKAGE_INIT_POS);
            claw_pivot.setPosition(Constants.CLAW_ASSEMBLY_INIT);
            turret.setPosition(Constants.TURRET_INIT);
            slider_claw_tilt.setPosition(Constants.SLIDER_TILT_INIT);
        }
    }


    public static class scoreAuto {
        Prepare SliderAction;
        Collect LinkageAction;
        Score ScoreAction;

        public scoreAuto(Prepare SliderAction, Collect LinkageAction, Score ScoreAction) {
            this.ScoreAction = ScoreAction;
            this.LinkageAction = LinkageAction;
            this.SliderAction = SliderAction;
        }


        public class HighChamber implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ScoreAction.placeOnHighChamber();
                return false;
            }
        }

        public class HighBasket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ScoreAction.placeInHighBasket();
                return false;
            }
        }

        public class BasketPreload implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ScoreAction.BasketPreload();
                return false;
            }
        }

        public class Place implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ScoreAction.score();
                return false;
            }
        }


            public class TakeFromHuman implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    SliderAction.takeFromHuman();
                    return false;
                }
            }

            public class TakeFromGround implements Action {

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    LinkageAction.takePos();
                    return false;
                }
            }

            public Action Chamber() {
                return new HighChamber();
            }

            public Action Basket() {
                return new HighBasket();
            }

            public Action TakeFromHuman() {
                return new TakeFromHuman();
            }

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


