package org.firstinspires.ftc.teamcode.teleop;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.commands.complex.IntakeRetract;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.Names;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;
@TeleOp
public class colorSensorTest extends OpMode{
    private ColorSensor colorSensor1;
    private ColorSensor colorSensor2;

    private RevBlinkinLedDriver lights;

    @Override
    public void init() {
        colorSensor1 = hardwareMap.get(ColorSensor.class, "color1");
        colorSensor2 = hardwareMap.get(ColorSensor.class, "color2");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        lights.resetDeviceConfigurationForOpMode();

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }


    @Override
    public void loop() {
        if(gamepad1.y){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
        if(colorSensor1.red() >= 150){
            telemetry.addData("red", colorSensor1.red());
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }else{
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
        telemetry.update();
    }
}
