package org.firstinspires.ftc.teamcode.utils;

public class Constants {
    public enum Names {
        frontLeft,
        frontRight,
        backLeft,
        backRight,
        leftOuttake,
        rightOuttake,
        intakeExtendo,
        slurp,
        door,
        intakePivot,
        intakeArm,
        outtakeArm,
        outtakePivot,
        clawPivot,
        claw,
    }

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
            case clawPivot: return      "clawPivot";
            case claw: return           "IDK3";
            default: return "";
        }
    }
}
