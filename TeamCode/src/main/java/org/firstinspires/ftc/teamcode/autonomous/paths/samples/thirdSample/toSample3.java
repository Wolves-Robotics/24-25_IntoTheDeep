package org.firstinspires.ftc.teamcode.autonomous.paths.samples.thirdSample;

import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Point;

public class toSample3 {
    public static Path getPath() {
        Path path  = new Path(new BezierLine(
                new Point(14.100, 129.600, Point.CARTESIAN),
                new Point(33.500, 128.000, Point.CARTESIAN)
        ));
        path.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(50));
        return path;
    }
}
