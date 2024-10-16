package org.firstinspires.ftc.teamcode.autonomous.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.collections.Color;
import org.firstinspires.ftc.teamcode.autonomous.collections.EnumMaps;
import org.firstinspires.ftc.teamcode.autonomous.collections.StartPos;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.HashMap;

@Autonomous
abstract public class BaseAuto extends OpMode {
    private EnumMaps enumMaps;
    private RobotHardware robotHardware;
    private Follower follower;
    private ElapsedTime elapsedTime;
    private Color color;
    private StartPos startPos;
    private int switchNumber;
    private double timeOffset;

    @Override
    public void init() {
        enumMaps = new EnumMaps();
        color = Color.RED;
        startPos = StartPos.RIGHT;
        switchNumber = 0;
        timeOffset = 0;
        elapsedTime = new ElapsedTime();
        robotHardware = new RobotHardware(hardwareMap, color);
    }

    @Override
    public void init_loop() {
        if (timeLimit(gamepad1.dpad_down)) switchNumber++;
        if (timeLimit(gamepad1.dpad_up)) switchNumber--;
        switchNumber %= 3;

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
        telemetry.addData("switch number", switchNumber);
        telemetry.addData("Selected", enumMaps.switchMap.get(switchNumber));
        telemetry.addData("Color", enumMaps.colorMap.get(color));
        telemetry.addData("Start position", enumMaps.startPosMap.get(startPos));
        telemetry.addData("Time offset", timeOffset);
        telemetry.update();
    }

    @Override
    public void loop() {
    }

    private boolean timeLimit(boolean bool) {
        boolean time = false;
        if (0.25 > elapsedTime.seconds()) {
            time = true;
            elapsedTime.reset();
        }
        return bool && time;
    }

    private void switchColorAndStartPos() {
        switch (switchNumber) {
            case 0:
                switch (color) {
                    case RED:
                        color = Color.BLUE;
                    case BLUE:
                        color = Color.RED;
                }
            case 1:
                switch (startPos) {
                    case LEFT:
                        startPos = StartPos.RIGHT;
                    case RIGHT:
                        startPos = StartPos.LEFT;
                }
        }
    }
}
