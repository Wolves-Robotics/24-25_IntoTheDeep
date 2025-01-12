package org.firstinspires.ftc.teamcode.utils;

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
            case transferColor: return  "color2";

            case imu: return            "imu";

            case lights: return         "led";
            default: return "";
        }
    }
    public static final double ip = 0.018, ii = 0.175, id = 0.0009;
    public static final double op = 0.005, oi = 0.15, od = 0.00012, of = 0.05;

    public static final int intakeMinTarget = 0, intakeMaxTarget = 400;
    public static final int outtakeMinTarget = 0, outtakeMaxTarget = 1940;
}
