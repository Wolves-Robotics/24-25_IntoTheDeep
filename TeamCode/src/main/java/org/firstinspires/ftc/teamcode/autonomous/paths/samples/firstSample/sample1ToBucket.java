package org.firstinspires.ftc.teamcode.autonomous.paths.samples.firstSample;

import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Point;

public class sample1ToBucket {
    public static Path getPath() {
        Path path = new Path(new BezierLine(
                new Point(34.000, 109.100, Point.CARTESIAN),
                new Point(14.190, 129.648, Point.CARTESIAN)
        ));
        path.setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(-45));
        return path;
    }
}
