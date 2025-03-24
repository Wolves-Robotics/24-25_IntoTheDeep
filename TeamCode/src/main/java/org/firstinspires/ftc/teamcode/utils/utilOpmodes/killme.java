package org.firstinspires.ftc.teamcode.utils.utilOpmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

@TeleOp
@Config
public class killme extends OpMode {
    private Limelight3A limelight3A;
    private MultipleTelemetry telemetryA;
    private ElapsedTime elapsedTime;

    private boolean left;
    public static int milliseconds = 500;

    @Override
    public void init() {
        RobotHardware.reset(hardwareMap);
        RobotHardware.getInstance().servoInit();

        elapsedTime = new ElapsedTime();

        DriveSubsystem.getInstance().setFollower(new Pose(0, 0, 0));
        left = true;
    }

    @Override
    public void start() {
        elapsedTime.reset();
    }

    @Override
    public void loop() {
        if (elapsedTime.milliseconds() > milliseconds) {
            DriveSubsystem.getInstance().getFollower().turnDegrees(20, left);
            elapsedTime.reset();
            left = !left;
        }


        telemetry.addData("heading", Math.toDegrees(DriveSubsystem.getInstance().getHeadimg()));
        telemetry.addData("milliseconds", elapsedTime.milliseconds());
        telemetry.update();
    }

    @Override
    public void stop() {
        RobotHardware.getInstance().interrupt();
    }
}
