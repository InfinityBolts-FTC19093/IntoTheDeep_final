package org.firstinspires.ftc.teamcode.autonomous.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

public class actionsManual {
    public static class Lift {
        private final DcMotorEx slider;
        private final slider_controller sliderController;

        public Lift(HardwareMap hardwareMap) {
            slider = hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_SLIDER);
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            sliderController = new slider_controller(slider, hardwareMap);
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

        public Action update() {
            return new Update();
        }

        public Action liftChamber() {return new MoveSlides(Constants.SLIDER_HIGH_CHAMBER);}

        public Action liftPlace() {
            return new MoveSlides(Constants.SLIDER_HIGH_CHAMBER + 500);
        }

        public Action liftBasket() {
            return new MoveSlides(Constants.SLIDER_HIGH_BUSKET);
        }

        public Action liftDown() {
            return new MoveSlides(Constants.SLIDER_DOWN);
        }

        public Action takeFromLinkage (){
            return new MoveSlides(Constants.SLIDER_TAKE_FORM_LINKAGE);
        }

        public Action liftPreload() {return new MoveSlides(Constants.SLIDER_PLACE_PRELOAD_AUTO);}

        public Action beforeLinkage () {
            return new MoveSlides(Constants.SLIDER_BEFORE_TAKE_FORM_LINKAGE);
        }
    }

    public static class SliderClaw {
        private final Servo slider_claw;

        public SliderClaw(HardwareMap hardwareMap) {
            slider_claw = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDER_CLAW);
        }

        public class Close implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slider_claw.setPosition(Constants.CLOSE_CLAW);
                return false;
            }
        }

        public class Open implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slider_claw.setPosition(Constants.OPEN_CLAW);
                return false;
            }
        }

        public Action close() {
            return new Close();
        }

        public Action open() {
            return new Open();
        }
    }


    public static class Claw {
        private final Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW);

        }

        public class Close implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(Constants.CLOSE_CLAW);
                return false;
            }
        }

        public class Open implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(Constants.OPEN_CLAW);
                return false;
            }
        }

        public Action close() {
            return new Close();
        }

        public Action open() {
            return new Open();
        }
    }


    public static class Linkage {
        private final Servo linkage;

        public Linkage(HardwareMap hardwareMap) {
            linkage = hardwareMap.get(Servo.class, HardwareConstants.ID_LINKAGE_SERVO);
        }

        public class Take implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                linkage.setPosition(Constants.LINKAGE_TAKE_POS);
                return false;
            }
        }

        public class Place implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                linkage.setPosition(Constants.LINKAGE_PLACE_IN_SLIDER);
                return false;
            }
        }

        public Action take() {
            return new Take();
        }

        public Action place() {
            return new Place();
        }

    }

    public static class ClawRotate {
        private final Servo rotate;

        public ClawRotate(HardwareMap hardwareMap) {
            rotate = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_ROTATE);
        }

        public class H implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotate.setPosition(Constants.ROTATE_INIT);
                return false;
            }
        }

        public class V implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotate.setPosition(Constants.ROTATE_TAKE_VERTICAL);
                return false;
            }
        }

        public Action horizontal() {
            return new H();
        }

        public Action vertical() {
            return new V();
        }

    }


    public static class SliderTilt {
        private final Servo slider_tilt;

        public SliderTilt(HardwareMap hardwareMap) {
            slider_tilt = hardwareMap.get(Servo.class, HardwareConstants.ID_SLIDER_CLAW_TILT);
        }

        public class Init implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slider_tilt.setPosition(Constants.SLIDER_TILT_INIT);
                return false;
            }
        }

        public class TakeLinkage implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slider_tilt.setPosition(Constants.SLIDER_TILT_TAKE_FROM_LINKAGE);
                return false;
            }
        }

        public class Basket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slider_tilt.setPosition(Constants.SLIDER_TILT_BASKET_AUTO);
                return false;
            }
        }

        public class Chamber implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slider_tilt.setPosition(Constants.SLIDER_TILT_PLACE_ON_HIGH_CHAMBER);
                return false;
            }
        }

        public class Human implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slider_tilt.setPosition(Constants.SLIDER_TILT_TAKE_FORM_HUMAN);
                return false;
            }
        }

        public class BeforeLinkage implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slider_tilt.setPosition(Constants.SLIDER_TILT_BEFORE_TAKE_FROM_LINKAGE);
                return false;
            }
        }

        public class Park implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slider_tilt.setPosition(Constants.SLIDER_TILT_PARK);
                return false;
            }
        }

        public Action init() {
            return new Init();
        }

        public Action take() {
            return new TakeLinkage();
        }

        public Action basket() {
            return new Basket();
        }

        public Action chamber() {
            return new Chamber();
        }

        public Action human() {
            return new Human();
        }

        public Action beforeLinkage() {
            return new BeforeLinkage();
        }

        public Action park() {return new Park();}
    }

    public static class Turret {
        private final Servo turret;

        public Turret(HardwareMap hardwareMap) {
            turret = hardwareMap.get(Servo.class, HardwareConstants.ID_TURRET);
        }

        public class Init implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                turret.setPosition(Constants.TURRET_INIT_AUTO);
                return false;
            }
        }

        public class Human implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                turret.setPosition(Constants.TURRET_TAKE_HUMAN);
                return false;
            }
        }

        public class Linkage implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                turret.setPosition(Constants.TURRET_TAKE_FROM_LINKAGE);
                return false;
            }
        }

        public class Place implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                turret.setPosition(Constants.TURRET_PLACE);
                return false;
            }
        }

        public Action init() {
            return new Init();
        }

        public Action human() {
            return new Human();
        }

        public Action take() {
            return new Linkage();
        }

        public Action place() {
            return new Place();
        }
    }

    public static class Pivot {
        private final Servo pivot;

        public Pivot (HardwareMap hardwareMap) {
            pivot = hardwareMap.get(Servo.class, HardwareConstants.ID_PIVOT);
        }
        public class Init implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                pivot.setPosition(Constants.CLAW_ASSEMBLY_INIT);
                return false;
            }
        }

        public class TakeGround implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                pivot.setPosition(Constants.CLAW_PIVOT_TAKE);
                return false;
            }
        }

        public Action init() {return new Init();}
        public Action take() {return new TakeGround();}
    }


    public static class ClawTilt {
        private final Servo tilt;

        public ClawTilt(HardwareMap hardwareMap) {
            tilt = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_TILT);
        }

        public class Init implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                tilt.setPosition(Constants.TILT_INIT);
                return false;
            }
        }

        public class TiltTake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                tilt.setPosition(Constants.TILT_TAKE);
                return false;
            }
        }

        public class TiltPlace implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                tilt.setPosition(Constants.TILT_PLACE_IN_SLIDER);
                return false;
            }
        }

        public class BeforeTake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                tilt.setPosition(Constants.TILT_BEFORE_TAKE_AUTO);
                return false;
            }
        }

        public Action init() {
            return new Init();
        }

        public Action take() {
            return new TiltTake();
        }

        public Action place() {
            return new TiltPlace();
        }

        public Action beforeTake() {
            return new BeforeTake();
        }
    }

    public static class Update {
        Servo claw, claw_tilt, linkage, claw_rotate, claw_pivot, slider_claw, slider_claw_tilt, turret;
        DcMotorEx slider;
        slider_controller sliderController;
        claw_controller clawController;
        sliderClaw_controller sliderClawController;
        clawRotate_controller clawRotateController;
        linkage_controller linkageController;

        public Update(HardwareMap hardwareMap) {
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


            sliderController = new slider_controller(slider, hardwareMap);
            clawController = new claw_controller(claw);
            sliderClawController = new sliderClaw_controller(slider_claw);
            clawRotateController = new clawRotate_controller(claw_rotate);
            linkageController = new linkage_controller(linkage);

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
    }
}




