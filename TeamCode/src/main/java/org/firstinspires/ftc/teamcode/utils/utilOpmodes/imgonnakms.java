package org.firstinspires.ftc.teamcode.utils.utilOpmodes;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.outtake.OpenClaw;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

@TeleOp
public class imgonnakms extends OpMode {
    private Limelight3A limelight3A;
    private int index;
    private double x;

    @Override
    public void init() {
        RobotHardware.reset(hardwareMap);
        RobotHardware.getInstance().servoInit();
        IntakeSubsystem.getInstance().bucketHover();

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.start();

        index = 0;

        DriveSubsystem.getInstance().setFollower(new Pose(0, 0));
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        if (index == 0) {
            LLResult latest = limelight3A.getLatestResult();
            if (latest.isValid()) {
                index = 1;
                x = latest.getTx();
                DriveSubsystem.getInstance().getFollower().holdPoint(
                        new Point(
                                DriveSubsystem.getInstance().getXPos(),
                                DriveSubsystem.getInstance().getYPos()-x*0.23
                        ),
                        DriveSubsystem.getInstance().getHeadimg()
                );
            }
        } else if (index == 1) {
        }
    }

    @Override
    public void stop() {
        RobotHardware.getInstance().interrupt();
    }
}
