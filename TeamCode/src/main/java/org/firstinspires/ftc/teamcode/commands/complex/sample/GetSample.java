package org.firstinspires.ftc.teamcode.commands.complex.sample;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.collections.Color;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.Names;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

public class GetSample extends CommandBase {
    private double time;

    private ElapsedTime elapsedTime, turnTime;

    private Color color;

    private boolean extended = false, wrongColor = false,
                    left = true;

    public GetSample(double _time, Color _color) {
        time = _time;
        color = _color;

        elapsedTime = new ElapsedTime();
        turnTime = new ElapsedTime();
    }

    @Override
    public void initialize() {
        elapsedTime.reset();
        turnTime.reset();

        IntakeSubsystem.getInstance().bucketDown();
        IntakeSubsystem.getInstance().slurpForward();
    }

    @Override
    public void execute() {
        if (!extended && elapsedTime.seconds() > 0.1) {
            IntakeSubsystem.getInstance().setTarget(350);
            extended = true;
        }

        if ((RobotHardware.getInstance().isBlue(Names.intakeColor) && color == Color.Red) || (RobotHardware.getInstance().isRed(Names.intakeColor) && color == Color.Blue)) {
            IntakeSubsystem.getInstance().slurpBackward();
            wrongColor = true;
        }

        if (turnTime.milliseconds() > 50) {
            DriveSubsystem.getInstance().getFollower().turnDegrees(20, left);
            left = !left;
            turnTime.reset();
        }

//        if (DriveSubsystem.getInstance().atParametricEnd()) {
//            DriveSubsystem.getInstance().getFollower().turnDegrees(20, left);
//            left = !left;
//        }
    }

    @Override
    public boolean isFinished() {
        return (RobotHardware.getInstance().isYellow(Names.intakeColor) ||
                (RobotHardware.getInstance().isRed(Names.intakeColor) && color == Color.Red) ||
                (RobotHardware.getInstance().isBlue(Names.intakeColor) && color == Color.Blue)) ||
                (elapsedTime.seconds() > time) ||
                (wrongColor);
    }
}
