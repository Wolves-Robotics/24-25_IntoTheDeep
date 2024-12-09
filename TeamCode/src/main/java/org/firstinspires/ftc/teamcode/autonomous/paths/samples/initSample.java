package org.firstinspires.ftc.teamcode.autonomous.paths.samples;

import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Point;

public class initSample {
    public static Path getPath() {
        Path path = new Path(new BezierCurve(
                new Point(7.256, 111.910, Point.CARTESIAN),
                new Point(17.577, 108.363, Point.CARTESIAN),
                new Point(14.674, 129.648, Point.CARTESIAN)
        ));
        path.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45));
        return path;
    }
}
