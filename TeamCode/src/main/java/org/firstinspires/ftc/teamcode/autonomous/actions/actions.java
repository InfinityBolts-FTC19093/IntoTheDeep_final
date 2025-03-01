package org.firstinspires.ftc.teamcode.autonomous.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

public class actions {
    public static class Auto {
        private final Servo claw, claw_tilt, linkage, claw_rotate, claw_pivot, slider_claw, slider_claw_tilt, turret;
        private final DcMotorEx slider;
        private final claw_controller clawController;
        private final sliderClaw_controller sliderClawController;
        private final clawRotate_controller clawRotateController;
        private final linkage_controller linkageController;
        private final slider_controller sliderController;
        private final PrepareAuto SliderAction;
        private final CollectAuto LinkageAction;
        private final ScoreAuto scoreAction;
        private Timing.Timer timer;

        public Auto(HardwareMap hardwareMap) {
            slider = hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_SLIDER);
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            claw = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW);
            slider_claw = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDER_CLAW);
            claw_rotate = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_ROTATE);
            claw_tilt = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_TILT);
            linkage = hardwareMap.get(Servo.class, HardwareConstants.ID_LINKAGE_SERVO);
            claw_pivot = hardwareMap.get(Servo.class, HardwareConstants.ID_PIVOT);
            turret = hardwareMap.get(Servo.class, HardwareConstants.ID_TURRET);
            slider_claw_tilt = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDER_CLAW_TILT);

            clawController = new claw_controller(claw);
            sliderClawController = new sliderClaw_controller(slider_claw);
            clawRotateController = new clawRotate_controller(claw_rotate);
            linkageController = new linkage_controller(linkage);
            sliderController = new slider_controller(slider, hardwareMap);

            SliderAction = new PrepareAuto(slider_claw, slider_claw_tilt, turret, slider, claw);
            LinkageAction = new CollectAuto(claw, claw_tilt, linkage, claw_rotate, claw_pivot);
            scoreAction = new ScoreAuto(claw, claw_tilt, linkage, claw_rotate, claw_pivot, slider_claw, slider_claw_tilt, turret, slider, LinkageAction, SliderAction);

            actionsManual.Lift lift = new actionsManual.Lift(hardwareMap);
            actionsManual.SliderClaw sliderClaw = new actionsManual.SliderClaw(hardwareMap);
            actionsManual.Linkage linkage = new actionsManual.Linkage(hardwareMap);
            actionsManual.Claw claw = new actionsManual.Claw(hardwareMap);
            actionsManual.ClawRotate clawRotate = new actionsManual.ClawRotate(hardwareMap);
            actionsManual.SliderTilt sliderTilt = new actionsManual.SliderTilt(hardwareMap);
            actionsManual.Turret turret = new actionsManual.Turret(hardwareMap);
            actionsManual.ClawTilt clawTilt = new actionsManual.ClawTilt(hardwareMap);
            actionsManual.Pivot pivot = new actionsManual.Pivot(hardwareMap);
            actionsManual.Update updateAuto = new actionsManual.Update(hardwareMap);

        }

        public class UpdateAll implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                clawController.update();
                clawRotateController.update();
                sliderClawController.update();
                linkageController.update();
                sliderController.update();
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
            turret.setPosition(Constants.TURRET_INIT_AUTO);
            slider_claw_tilt.setPosition(Constants.SLIDER_TILT_INIT);
        }


//    public static class scoreAuto {
//        PrepareAuto SliderAction;
//        CollectAuto LinkageAction;
//        ScoreAuto scoreAction;
//        slider_controller sliderController;
//        Timing.Timer timer;
//
//        public scoreAuto(PrepareAuto SliderAction, CollectAuto LinkageAction, ScoreAuto ScoreAction) {
//            this.scoreAction = ScoreAction;
//            this.LinkageAction = LinkageAction;
//            this.SliderAction = SliderAction;
//        }

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
                scoreAction.take();
                return false;
            }
        }

        public Action Chamber() {
            return new HighChamber();
        }

        public Action Basket() {return new HighBasket();}

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


