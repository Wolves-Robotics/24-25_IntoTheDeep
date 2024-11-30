package org.firstinspires.ftc.teamcode.autonomous.paths.samples.firstSample;

import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Point;

public class toSample1 {
    public static Path getPath() {
        Path path = new Path(new BezierCurve(
                new Point(39.000, 82.000, Point.CARTESIAN),
                new Point(34.000, 90.500, Point.CARTESIAN),
                new Point(34.000, 109.100, Point.CARTESIAN)
        ));
        path.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(46));
        return path;
    }
}
