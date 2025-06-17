package org.firstinspires.ftc.teamcode.utils.utilOpmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Names;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

@TeleOp(group = "Utils")
@Config
public class ColorSensorTest extends OpMode {
    private RobotHardware robotHardware;
    public static Names colorSensor;
    private RevBlinkinLedDriver lights;
    private int totalRed = 0, numRed = 0, highestRed = -999, lowestRed = 999,
                totalBlue = 0, numBlue = 0, highestBlue = -999, lowestBlue = 999,
                totalGreen = 0, numGreen = 0, highestGreen = -999, lowestGreen = 999;
    @Override
    public void init() {
        RobotHardware.reset(hardwareMap);
        robotHardware = RobotHardware.getInstance();
        colorSensor = Names.transferColor;
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "led");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            int red = robotHardware.getRed(colorSensor);
            totalRed += red;
            numRed++;
            if (red > highestRed) highestRed = red;
            if (red < lowestRed) lowestRed = red;

            int blue = robotHardware.getBlue(colorSensor);
            totalBlue += blue;
            numBlue++;
            if (blue > highestBlue) highestBlue = blue;
            if (blue < lowestBlue) lowestBlue = blue;

            int green = robotHardware.getGreen(colorSensor);
            totalGreen += green;
            numGreen++;
            if (green > highestGreen) highestGreen = green;
            if (green < lowestGreen) lowestGreen = green;

            telemetry.addData("Average red", totalRed/numRed);
            telemetry.addData("Highest red", highestRed);
            telemetry.addData("Lowest red", lowestRed);

            telemetry.addData("Average blue", totalBlue/numBlue);
            telemetry.addData("Highest blue", highestBlue);
            telemetry.addData("Lowest blue", lowestBlue);


            telemetry.addData("Average green", totalGreen/numGreen);
            telemetry.addData("Highest green", highestGreen);
            telemetry.addData("Lowest green", lowestGreen);

            telemetry.addData("Distance", robotHardware.getDistance(Names.intakeColor));

            telemetry.update();
        }

        if (gamepad1.x) {
            totalRed = 0; numRed = 0; highestRed = -999999; lowestRed = 999999;
            totalBlue = 0; numBlue = 0; highestBlue = -999999; lowestBlue = 999999;
            totalGreen = 0; numGreen = 0; highestGreen = -999999; lowestGreen = 999999;

            telemetry.addData("Average red", 0);
            telemetry.addData("Highest red", highestRed);
            telemetry.addData("Lowest red", lowestRed);

            telemetry.addData("Average blue", 0);
            telemetry.addData("Highest blue", highestBlue);
            telemetry.addData("Lowest blue", lowestBlue);


            telemetry.addData("Average green", 0);
            telemetry.addData("Highest green", highestGreen);
            telemetry.addData("Lowest green", lowestGreen);

            telemetry.addData("Distance", robotHardware.getDistance(Names.intakeColor));

            telemetry.update();
        }
    }
}
