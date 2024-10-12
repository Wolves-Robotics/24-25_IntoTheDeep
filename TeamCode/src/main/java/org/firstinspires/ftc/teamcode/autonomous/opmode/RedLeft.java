package org.firstinspires.ftc.teamcode.autonomous.opmode;

import org.firstinspires.ftc.teamcode.autonomous.enums.Color;
import org.firstinspires.ftc.teamcode.autonomous.enums.StartPos;

public class RedLeft extends BaseAuto{
    @Override
    void setVars() {
        color = Color.RED;
        startPos = StartPos.LEFT;
    }
}
