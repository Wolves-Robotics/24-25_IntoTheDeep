package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.collections.Color;
import org.firstinspires.ftc.teamcode.collections.teleop.TeleOps;

public class TeleOpSelection extends Thread {
    private Gamepad gamepad;
    private ElapsedTime elapsedTime;
    private Select select;

    private TeleOps teleOps;
    private Color color;

    private enum Select {
        TeleOp, Color
    }

    public TeleOpSelection(Gamepad _gamepad) {
        gamepad = _gamepad;

        elapsedTime = new ElapsedTime();

        select = Select.TeleOp;
        teleOps = TeleOps.LandonKota;
        color = Color.Red;

        start();
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            if (buttonTime(gamepad.dpad_up || gamepad.dpad_down)) {
                select = select == Select.TeleOp ? Select.Color : Select.TeleOp;
            }

            if (buttonTime(gamepad.dpad_left || gamepad.dpad_right)) {
                switch (select) {
                    case TeleOp:
                        teleOps = teleOps == TeleOps.LandonKota ? TeleOps.Kota : TeleOps.LandonKota;
                        break;
                    case Color:
                        color = color == Color.Red ? Color.Blue : Color.Red;
                        break;
                }
            }
        }
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Selection", select == Select.TeleOp ? "Tele Op" : "Color");
        telemetry.addData("Tele Op", teleOps == TeleOps.LandonKota ? "Landon and Kota" : "Kota");
        telemetry.addData("Color", color == Color.Red ? "Red" : "Blue");
    }

    private boolean buttonTime(boolean bool) {
        if (0.25 < elapsedTime.seconds() && bool) {
            elapsedTime.reset();
            return true;
        }
        return false;
    }

    public TeleOps getTeleOp() {
        return teleOps;
    }
    public Color getColor() {
        return color;
    }
}
