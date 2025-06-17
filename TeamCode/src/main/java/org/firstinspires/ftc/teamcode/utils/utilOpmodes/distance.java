package org.firstinspires.ftc.teamcode.utils.utilOpmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Names;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

@TeleOp
public class distance extends OpMode {

    private RobotHardware robotHardware;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        RobotHardware.reset(hardwareMap);
        robotHardware = RobotHardware.getInstance();
    }

    @Override
    public void loop() {
        telemetry.addData("distance", robotHardware.getDistance(Names.transferColor));
        telemetry.update();
    }
}
