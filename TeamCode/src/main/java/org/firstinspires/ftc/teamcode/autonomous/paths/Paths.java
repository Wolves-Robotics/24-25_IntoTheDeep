package org.firstinspires.ftc.teamcode.autonomous.paths;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.collections.Color;
import org.firstinspires.ftc.teamcode.autonomous.collections.StartPos;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.autonomous.selection.AutoSelection;

public class Paths extends Thread{
    private PathGen pathGen;
    private PathEnum pathEnum;
    private Color color;
    private StartPos startPos;
    private double timeOffset;
    private Follower follower;

    public Paths(HardwareMap hardwareMap) {
        pathGen = new PathGen();
        pathEnum = PathEnum.PARK;
        follower = new Follower(hardwareMap);
    }

    @Override
    public void run() {
        pathGen.generate(startPos);
        while (!Thread.currentThread().isInterrupted()) {
            follower.update();
            switch (pathEnum) {
                case PARK:
                    follower.followPath(pathGen.getParkPath());
                    break;
            }
        }
    }

    public void updateTelemetry(Telemetry telemetry) {

    }

    public void setVars(AutoSelection selection) {
        color = selection.getColor();
        startPos = selection.getStartPos();
        timeOffset = selection.getTimeOffset();
    }
}
