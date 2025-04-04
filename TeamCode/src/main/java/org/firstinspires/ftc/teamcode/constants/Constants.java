package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
@Config
public class Constants {

    public static int WAIT_FOR_LINKAGE_ACTION = 400;
    public static int WAIT_FOR_SLIDER_ACTION = 150;

    public enum BasketPos{
        HIGH_BASKET,
        LOW_BASKET
    }

    public enum ClawPos {
        OPEN_CLAW,
        CLOSE_CLAW
    }

    public enum SliderClawPos{
        OPEN_CLAW,
        CLOSE_CLAW
    }

    public enum SliderPos{
        SLIDER_DOWN,
        SLIDER_NOT_DOWN,
        SLIDER_HIGH,
        SLIDER_LOW
    }

    public enum LinkageActionPos {
        TAKE,
        PLACE_IN_SLIDER,
        TAKE2,
        INIT
    }

    public enum SliderActionPos {
        TAKE_FOR_LINKAGE,
        TAKE_FROM_HUMAN,
        PLACE_ON_CHAMBER,
        PLACE_IN_BUSKET,
        BEFORE_TAKE_FROM_LINKAGE,
        INIT
    }

    public enum ScorePos{
        TAKE,
        CHAMBER,
        BUSKET,
        INIT
    }

    public enum UnghiRobotPos{
        SUS,
        INTERMEDIARY,
        JOS
    }

    public enum ClawRotatePos{
        HORIZONTAL,
        VERTICAL,
        DIAGONAL_NEGATIV,
        DIAGONAL_POSITIV,
        INVERTED
    }

    public enum LinkagePos{
        INIT,
        AUTO,
        MANUAL
    }

    public static ClawPos currentClawPos = ClawPos.OPEN_CLAW;
    public static SliderPos currentSliderPos = SliderPos.SLIDER_DOWN;
    public static LinkageActionPos currentLinkageActionPos = LinkageActionPos.INIT;
    public static SliderActionPos currentSliderActionPos = SliderActionPos.INIT;
    public static SliderActionPos previousSliderActionPos = SliderActionPos.INIT;
    public static ScorePos currentScorePos = ScorePos.INIT;
    public static SliderClawPos currentSliderClawPos = SliderClawPos.OPEN_CLAW;
    public static UnghiRobotPos currentUnghiRobotPos = UnghiRobotPos.SUS;
    public static ClawRotatePos currentClawRotatePos = ClawRotatePos.HORIZONTAL;
    public static BasketPos currentBasketPos = BasketPos.HIGH_BASKET;
    public static LinkagePos currentLinkagePos = LinkagePos.INIT;


    /** SLIDER*/
    public static final int SLIDER_DOWN = -1;

    public static int SLIDER_TAKE_FORM_LINKAGE = 350;
    public static int SLIDER_BEFORE_TAKE_FORM_LINKAGE = 350;

    public static int SLIDER_LOW_CHAMBER = 0;
    public static int SLIDER_HIGH_CHAMBER = 240;

    public static int SLIDER_LOW_BUSKET = 800;
    public static int SLIDER_HIGH_BUSKET =2050;
    public static int SLIDER_HIGH_BUSKET_AUTO =1950;


    public static int SLIDER_ASCEND = 0;
    public static int SLIDER_LEV2_ASCEND = 0;

    public static double Slider_kPAuto = 0.01;

    public static double Slider_kP = 0.03;
    public static double Slider_kI = 0.0001;
    public static double Slider_kD = 0;


    /** PINPOINT OFFSETS*/
    public static double X_OFFSET = -4.8032;
    public static double Y_OFFSET = -0.5512;



    /** CLAW*/
    public static double OPEN_CLAW = 0;
    public static double CLOSE_CLAW = 1;

    /** SLIDER CLAW*/
    public static double OPEN_CLAW_SLIDER = 0.4;
    public static double CLOSE_CLAW_SLIDER = 0.8;

    /** Servo LINKAGE */
    public static double LINKAGE_INIT_POS = 0.1;
    public static double LINKAGE_MAX_POS = 0.69;
    public static double LINKAGE_TAKE_POS = 0.80;
    public static double LINKAGE_PLACE_IN_SLIDER = 0.06;
    public static double LINKAGE_INTERMEDIARY_POS = 0;

    /** Servo TILT */
    public static double TILT_INIT = 0;
    public static double TILT_TAKE = 0.9;
    public static double TILT_BEFORE_TAKE = 0.75;
    public static double TILT_AFTER_TAKE = 0.5;
    public static double TILT_BEFORE_TAKE_AUTO = 0.65;
    public static double TILT_PLACE_IN_SLIDER = 0.08;
    public static double TILT_THROW = 0.3;

    /** Servo ROTATE */
    public static double ROTATE_INIT = 0;
    public static double ROTATE_TAKE_HORIONTAL = 0;
    public static double ROTATE_TAKE_DIAGONAL_NEGATIV = 0.275;
    public static double ROTATE_TAKE_VERTICAL = 0.475;
    public static double ROTATE_TAKE_DIAGONAL_POSITIV = 0.72;
    public static double ROTATE_PLACE_IN_SLIDER = 0;
    public static double ROTATE_PLACE_IN_SLIDER_INVERTED = .9;

    /** Servo Claw ASSEMBLY*/
    public static double CLAW_ASSEMBLY_INIT = 0;
    public static double CLAW_ASSEMBLY_PLACE_IN_SLIDER = 0;
    public static double CLAW_ASSEMBLY_TAKE = 0;
    public static double CLAW_ASSEMBLY_TAKE_RIGHT = .35;
    public static double CLAW_PIVOT_TAKE = 0.2;


    /** Servo TURRET */
    public static double TURRET_INIT_AUTO = .55;
    public static double TURRET_TAKE_HUMAN = 0.17;
    public static double TURRET_TAKE_FROM_LINKAGE = 0.17;
    public static double TURRET_PLACE = 0.17;
    public static double TURRET_BASKET_AUTO = 0.075;



    /** Servo Slider Claw Tilt */
    public static double SLIDER_TILT_INIT_BASKET = 0.84;
    public static double SLIDER_TILT_INIT_CHAMBER = 0;
    public static double SLIDER_TILT_TAKE_FROM_LINKAGE = 0.9;
    public static double SLIDER_TILT_PLACE_IN_HIGH_BUSKET = 0.225;
    public static double SLIDER_TILT_BASKET_AUTO = 0.223;
    public static double SLIDER_TILT_PLACE_IN_LOW_BUSKET = 0.2;
    public static double SLIDER_TILT_PLACE_ON_HIGH_CHAMBER = 0.6;
    public static double SLIDER_TILT_PLACE_ON_LOW_CHAMBER = 0;
    public static double SLIDER_TILT_TAKE_FORM_HUMAN = 0.115;
    public static double SLIDER_TILT_BEFORE_TAKE_FROM_LINKAGE = 0.75;
    public static double SLIDER_TILT_BEFORE_TAKE_FORM_HUMAN = 0.2;
    public static double SLIDER_TILT_PARK = 0.57;

    /** Servo Unghi Robot */
    public static double UNGHI_ROBOT_SUS = 0;
    public static double UNGHI_ROBOT_INTERMEDIARY = 0;
    public static double UNGHI_ROBOT_JOS = 0;
}