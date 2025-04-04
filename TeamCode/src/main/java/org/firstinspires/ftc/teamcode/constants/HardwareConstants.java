package org.firstinspires.ftc.teamcode.constants;


/**
 *  Control:
 *      Motor:
 *          0: lf
 *          1: lb
 *          2: rf
 *      Servo:
 *          0: claw
 *          3: SC
 *          4: RCA
 *          5: SCR
 *
 *          bus
 *          0: imu (BHI)
 *          1: pinpoint
 *          3: CCC
 *
 *  Expantion:
 *      Motor:
 *          0: rb
 *          1: slider
 *      Servo:
 *          2: CT
 *          3: linkage
 *          4: SCT
 *          5: CR
 *
 *          bus
 *          1: CRC
 * */

public class HardwareConstants {

    public static final String ID_LEFT_FRONT = "lf";
    public static final String ID_RIGHT_FRONT = "rf";
    public static final String ID_LEFT_BACK = "lb";
    public static final String ID_RIGHT_BACK = "rb";

    public static final String ID_SLIDER = "slider";

    //Servo

    public static final String ID_LINKAGE_SERVO = "linkage";

    public static final String ID_PIVOT = "RCA";
    public static final String ID_CLAW_TILT = "CT";
    public static final String ID_CLAW_ROTATE = "CR";
    public static final String ID_CLAW = "claw";

    public static final String ID_SLIDER_CLAW = "SC";
    public static final String ID_TURRET = "SCR";
    public static final String ID_SLIDER_CLAW_TILT = "SCT";

    public static final String ID_MODIFICA_UNGHI_ROBOT = "MUR";

    public static final String ID_ODO_STANGA =  ID_RIGHT_FRONT;
    public static final String ID_ODO_DREAPTA = ID_LEFT_FRONT;
    public static final String ID_ODO_MIJLOC =  ID_LEFT_BACK;

    public static final String ID_COLOR_CENTER_CLAW = "CCC";
    public static final String ID_COLOR_ROTATE_CLAW = "CRC";

    public static final String ID_PINPOINT = "pinpoint";



}
