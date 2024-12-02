package org.firstinspires.ftc.teamcode.teleop.singleThings;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestIntakeControl extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        telemetry.addData("Finger 1 x", gamepad1.touchpad_finger_1_x);
        telemetry.update();
    }
}
