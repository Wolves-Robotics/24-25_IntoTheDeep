package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.utils.collections.Names;

public class Constants {
    public static String getStringName(Names name) {
        switch (name) {
            case frontLeft: return      "frontLeft";
            case frontRight: return     "frontRight";
            case backLeft: return       "backLeft";
            case backRight: return      "backRight";
            case leftOuttake: return    "leftOuttake";
            case rightOuttake: return   "rightOuttake";
            case intakeExtendo: return  "intakeExtendo";
            case slurp: return          "slurp";

            case door: return           "IDK1";
            case intakePivot: return    "IDK2";
            case intakeArm: return      "intakeArm";
            case outtakeArm: return     "outtakeArm";
            case outtakePivot: return   "IDK4";
            case claw: return           "IDK3";

            case intakeColor: return    "color1";
            case transferDistance: return  "color2";

            case pinpoint: return       "pinpoint";

            case lights: return         "led";
            default: return "";
        }
    }

    public static boolean getHardwareReversed(Names name) {
        switch (name) {
            case frontLeft: return      true;
            case frontRight: return     false;
            case backLeft: return       true;
            case backRight: return      false;
            case leftOuttake: return    true;
            case rightOuttake: return   false;
            case intakeExtendo: return  false;
            case slurp: return          false;

            case door: return           false;
            case intakePivot: return    false;
            case intakeArm: return      false;
            case outtakeArm: return     false;
            case outtakePivot: return   false;
            case claw: return           false;

            default: return             false;
        }
    }

    public static final double IP = 0.018, II = 0.175, ID = 0.0009;
    public static final double OP = 0.005, OI = 0.19, OD = 0.00012, OF = 0.05;

    public static final int INTAKE_MIN_TARGET = 0, INTAKE_MAX_TARGET = 400;
    public static final int OUTTAKE_MIN_TARGET = 0, OUTTAKE_MAX_TARGET = 1940;

    //TODO: add all servo and motor positions and powers
    public static final double CLAW_OPEN = 0.35, CLAW_CLOSE = 0;

    public static final double AUTO_START_ARM_POS = 0.23, AUTO_START_PIVOT_POS = 0.4;
}