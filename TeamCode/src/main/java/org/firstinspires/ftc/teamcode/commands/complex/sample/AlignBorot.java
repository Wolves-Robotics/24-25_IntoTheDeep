package org.firstinspires.ftc.teamcode.commands.complex.sample;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

public class AlignBorot extends CommandBase {

    private Limelight3A limelight;
    private double tx;
    private int index = 0;
    private ElapsedTime elapsedTime;

    public AlignBorot() {
        limelight = RobotHardware.getInstance().getHardwareMap().get(Limelight3A.class, "limelight");
        limelight.start();
    }
    @Override
    public void execute() {
        switch (index) {
            case 0:
                LLResult latest = limelight.getLatestResult();
                if (latest.isValid()) {
                    index = 1;
                    tx = latest.getTx();
                    DriveSubsystem.getInstance().getFollower().holdPoint(
                            new Point(
                                    DriveSubsystem.getInstance().getXPos(),
                                    DriveSubsystem.getInstance().getYPos() + 6
                            ),
                            DriveSubsystem.getInstance().getHeadimg() + tx*0.5
                    );
                }
                break;
            case 1:

                break;
        }
    }
}
