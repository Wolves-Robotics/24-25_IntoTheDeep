package org.firstinspires.ftc.teamcode.autonomous.paths;

import org.firstinspires.ftc.teamcode.autonomous.collections.Color;
import org.firstinspires.ftc.teamcode.autonomous.collections.StartPos;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Point;

public class PathGen {
    private Path parkPath;

    public void generate(StartPos startPos) {
        parkPath = new Path(
                new BezierLine(
                        new Point(8.546, 44.800, Point.CARTESIAN),
                        new Point(8.546, 17.800, Point.CARTESIAN)
                )
        );
        parkPath.setConstantHeadingInterpolation(0);
    }

    public Path getParkPath() {
        return parkPath;
    }
}
