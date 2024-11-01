package org.firstinspires.ftc.teamcode.autonomous.selection;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.collections.Color;
import org.firstinspires.ftc.teamcode.autonomous.collections.EnumMaps;
import org.firstinspires.ftc.teamcode.autonomous.collections.StartPos;

public class AutoSelection extends Thread {
    private EnumMaps enumMaps;
    private ElapsedTime elapsedTime;
    private Color color;
    private StartPos startPos;
    private int switchNumber;
    private double timeOffset;
    private Gamepad gamepad1;

    public AutoSelection(Gamepad _gamepad1) {
        enumMaps = new EnumMaps();
        color = Color.RED;
        startPos = StartPos.RIGHT;
        switchNumber = 0;
        timeOffset = 0;
        elapsedTime = new ElapsedTime();
        gamepad1 = _gamepad1;
    }

    public void updateTelem(Telemetry telemetry) {
        telemetry.addData("Selected", enumMaps.switchMap.get(switchNumber));
        telemetry.addData("Color", enumMaps.colorMap.get(color));
        telemetry.addData("Start position", enumMaps.startPosMap.get(startPos));
        telemetry.addData("Time offset", timeOffset);
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            if (timeLimit(gamepad1.dpad_down)) switchNumber--;
            else if (timeLimit(gamepad1.dpad_up)) switchNumber++;
            switchNumber = (switchNumber + 3) % 3;

            if (timeLimit(gamepad1.dpad_right)) {
                switchColorAndStartPos();
                if (switchNumber == 2) {
                    timeOffset += 0.5;
                }
            } else if (timeLimit(gamepad1.dpad_left)) {
                switchColorAndStartPos();
                if (switchNumber == 2) {
                    timeOffset -= 0.5;
                }
            }
            timeOffset = Math.max(timeOffset, 0);
        }
    }

    public Color getColor() {
        return color;
    }
    public StartPos getStartPos() {
        return startPos;
    }
    public double getTimeOffset() {
        return timeOffset;
    }

    private boolean timeLimit(boolean bool) {
        boolean time;
        if (0.25 < elapsedTime.seconds() && bool) {
            time = true;
            elapsedTime.reset();
        } else time = false;
        return time;
    }

    private void switchColorAndStartPos () {
        switch (switchNumber) {
            case 0:
                if (color == Color.RED) {
                    color = Color.BLUE;
                } else {
                    color = Color.RED;
                }
                break;
            case 1:
                if (startPos == StartPos.LEFT) {
                    startPos = StartPos.RIGHT;
                } else {
                    startPos = StartPos.LEFT;
                }
                break;
        }
    }
}
