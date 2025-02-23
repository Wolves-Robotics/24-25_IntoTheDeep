package org.firstinspires.ftc.teamcode.utils.utilOpmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

@TeleOp
@Config
public class killme extends OpMode {
    private Limelight3A limelight3A;
    private MultipleTelemetry telemetryA;

    public static double kpHorizontal = 0., tolerance = 0.;
    private boolean left;

    @Override
    public void init() {
        RobotHardware.reset(hardwareMap);
        RobotHardware.getInstance().servoInit();

        DriveSubsystem.getInstance().setFollower(new Pose(0, 0, 0));
        left = true;
    }

    @Override
    public void start() {
        DriveSubsystem.getInstance().getFollower().turnDegrees(30, true);
        left = !left;
    }

    @Override
    public void loop() {
        if (Math.toDegrees(DriveSubsystem.getInstance().getHeadimg()) > 20 && !left) {
            DriveSubsystem.getInstance().getFollower().turnDegrees(30, left);
            left = !left;
        }
        if (Math.toDegrees(DriveSubsystem.getInstance().getHeadimg()) > 340 && left) {
            DriveSubsystem.getInstance().getFollower().turnDegrees(30, left);
            left = !left;
        }
        telemetry.addData("heading", Math.toDegrees(DriveSubsystem.getInstance().getHeadimg()));
        telemetry.update();
    }

    @Override
    public void stop() {
        RobotHardware.getInstance().interrupt();
    }
}
