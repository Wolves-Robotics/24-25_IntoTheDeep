package org.firstinspires.ftc.teamcode.auto.pedro.localization.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.pedro.localization.localizers.ThreeWheelIMULocalizer;

@TeleOp
public class EncoderDirectionCalibration extends OpMode {
    ThreeWheelIMULocalizer threeWheelIMULocalizer;

    @Override
    public void init() {
        threeWheelIMULocalizer = new ThreeWheelIMULocalizer(hardwareMap);
    }

    @Override
    public void loop() {
        threeWheelIMULocalizer.leftEncoder.update();
        threeWheelIMULocalizer.rightEncoder.update();
        threeWheelIMULocalizer.strafeEncoder.update();
        telemetry.addData("Left", threeWheelIMULocalizer.leftEncoder.getDeltaPosition());
        telemetry.addData("right", threeWheelIMULocalizer.rightEncoder.getDeltaPosition());
        telemetry.addData("strafe", threeWheelIMULocalizer.strafeEncoder.getDeltaPosition());
        telemetry.update();
    }
}
