package org.firstinspires.ftc.teamcode.teleop.singleThings;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class IntakeRun extends OpMode {
    Motor motor;

    @Override
    public void init() {
        motor = hardwareMap.get(Motor.class, "intake");
    }

    @Override
    public void loop() {
        if (gamepad1.a) motor.set(1);
        else motor.set(0);
    }
}
