package org.firstinspires.ftc.teamcode.commands.complex.sample;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.Names;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

public class TransferDistance extends CommandBase{
    private ElapsedTime elapsedTime;
    private boolean condition = false;
    private RobotHardware r;


    @Override
    public void initialize(){
        r = RobotHardware.getInstance();
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }
    public void execute(){
        if(r.getDistance(Names.transferColor) < 1.5){
            condition = true;
        }
        else if(elapsedTime.seconds() > 1.5){
            condition = true;
        }
    }
    public boolean isFinished(){
        return condition;
    }
}
