package org.firstinspires.ftc.teamcode.autonomous.paths.samples.secondSample;

import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Point;

public class sample2ToBucket {
    public static Path getPath() {
        Path path = new Path(new BezierLine(
                new Point(27.000, 130.100, Point.CARTESIAN),
                new Point(14.100, 129.600, Point.CARTESIAN)
        ));
        path.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));
        return path;
    }
}
