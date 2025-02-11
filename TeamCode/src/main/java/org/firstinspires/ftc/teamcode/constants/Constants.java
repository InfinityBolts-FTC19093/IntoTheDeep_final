package org.firstinspires.ftc.teamcode.constants;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;


@Config
public class Constants {

    public static final int WAIT_FOR_LINKAGE_ACTION = 300;
    public static final int WAIT_FOR_SLIDER_ACTION = 350;

    static RobotMap robot = new RobotMap(hardwareMap);

    public enum RobotDriveStatus{
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    public enum ClawPos{
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
        INIT
    }

    public enum SliderActionPos {
        TAKE_FOR_LINKAGE,
        PLACE_ON_CHAMBER,
        PLACE_IN_BUSKET,
        INIT
    }

    public enum ScorePos{
        TAKE,
        PLACE_ON_HIGH_CHAMBER,
        PLACE_ON_HIGH_BUSKET,
        PLACE_ON_LOW_CHAMBER,
        PLACE_ON_LOW_BUSKET,
        INIT
    }

    public enum SliderClawPos{
        OPEN_CLAW,
        CLOSE_CLAW
    }

    public enum UnghiRobotPos{
        SUS,
        INTERMEDIARY,
        JOS
    }

    public static RobotDriveStatus currentRobotDriveStatus = RobotDriveStatus.ROBOT_CENTRIC;
    public static ClawPos currentClawPos = ClawPos.OPEN_CLAW;
    public static SliderPos currentSliderPos = SliderPos.SLIDER_DOWN;
    public static LinkageActionPos currentLinkageActionPos = LinkageActionPos.INIT;
    public static SliderActionPos currentSliderActionPos = SliderActionPos.INIT;
    public static ScorePos currentScorePos = ScorePos.INIT;
    public static SliderClawPos currentSliderClawPos = SliderClawPos.CLOSE_CLAW;
    public static UnghiRobotPos currentUnghiRobotPos = UnghiRobotPos.SUS;


    /** SLIDER*/
    public static final int SLIDER_DOWN = 0;

    public static int SLIDER_TAKE_FORM_LINKAGE = 0;

    public static int SLIDER_LOW_CHAMBER = 0;
    public static int SLIDER_HIGH_CHAMBER = 0;

    public static int SLIDER_LOW_BUSKET = 0;
    public static int SLIDER_HIGH_BUSKET = 0;

    public static int SLIDER_ASCEND = 0;

    public static double Slider_kP = 0.1;
    public static double Slider_kI = 0.0004;
    public static double Slider_kD = 0;

    public static int SLIDER_PLACE_ON_CHAMBER = robot.slider.getCurrentPosition()-100;


    /** CLAW*/
    public static double OPEN_CLAW = 0;
    public static double CLOSE_CLAW = 0;

    /** Servo LINKAGE */
    public static double LINKAGE_INIT_POS = 0;
    public static double LINKAGE_MAX_POS = 0;
    public static double LINKAGE_TAKE_POS = 0;
    public static double LINKAGE_PLACE_IN_SLIDER = 0;
    public static double LINKAGE_INTERMEDIARY_POS = 0;

    /** Servo TILT */
    public static double TILT_INIT = 0;
    public static double TILT_TAKE = 0;
    public static double TILT_PLACE_IN_SLIDER = 0;

    /** Servo ROTATE */
    public static double ROTATE_INIT = 0;
    public static double ROTATE_TAKE = 0;
    public static double ROTATE_PLACE_IN_SLIDER = 0;


    /** Servo Slider Claw */
    public static double SLIDER_OPEN = 0;
    public static double SLIDER_CLOSE = 0;

    /** Servo Claw ASSEMBLY*/
    public static double CLAW_ASSEMBLY_INIT = 0;
    public static double CLAW_ASSEMBLY_PLACE_IN_SLIDER = 0;
    public static double CLAW_ASSEMBLY_TAKE = 0;


    /** Servo Slider Claw Rotate */
    public static double SLIDER_ROTATE_INIT = 0;
    public static double SLIDER_ROTATE_TAKE_HUMAN = 0;
    public static double SLIDER_ROTATE_TAKE_FROM_LINKAGE = 0;
    public static double SLIDER_ROTATE_PLACE = 0;

    /** Servo Slider Claw Tilt */
    public static double SLIDER_TILT_INIT = 0;
    public static double SLIDER_TILT_TAKE_FROM_LINKAGE = 0;
    public static double SLIDER_TILT_PLACE_IN_BUSKET = 0;
    public static double SLIDER_TILT_PLACE_ON_HIGH_CHAMBER = 0;
    public static double SLIDER_TILT_PLACE_ON_LOW_CHAMBER = 0;
    public static double SLIDER_TILT_TAKE_FORM_HUMAN = 0;



    /** Servo Unghi Robot */
    public static double UNGHI_ROBOT_SUS = 0;
    public static double UNGHI_ROBOT_INTERMEDIARY = 0;
    public static double UNGHI_ROBOT_JOS = 0;
}