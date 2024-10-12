package org.firstinspires.ftc.teamcode.autonomous.opmode;

import org.firstinspires.ftc.teamcode.autonomous.enums.Color;
import org.firstinspires.ftc.teamcode.autonomous.enums.StartPos;

public class BlueRight extends BaseAuto{
    @Override
    void setVars() {
        color = Color.BLUE;
        startPos = StartPos.RIGHT;
    }
}
