package org.firstinspires.ftc.teamcode.autonomous.collections;

import java.util.HashMap;

public class EnumMaps {
    public HashMap<Color, String> colorMap;
    public HashMap<StartPos, String> startPosMap;
    public HashMap<Integer, String> switchMap;

    public EnumMaps() {
        setSwitchMap();
        setColorMap();
        setStartPosMap();
    }

    private void setSwitchMap() {
        switchMap = new HashMap<>();
        switchMap.put(0, "Color");
        switchMap.put(1, "Start position");
    }

    private void setColorMap() {
        colorMap = new HashMap<>();
        colorMap.put(Color.RED, "Red");
        colorMap.put(Color.BLUE, "Blue");
    }

    private void setStartPosMap() {
        startPosMap = new HashMap<>();
        startPosMap.put(StartPos.LEFT, "Left");
        startPosMap.put(StartPos.RIGHT, "Right");
    }
}