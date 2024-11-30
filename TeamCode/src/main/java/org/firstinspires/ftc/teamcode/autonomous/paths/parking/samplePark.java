package org.firstinspires.ftc.teamcode.autonomous.paths.parking;

import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Point;

public class samplePark {
    public static Path getPath() {
        Path path = new Path(new BezierCurve(
                new Point(14.100, 129.600, Point.CARTESIAN),
                new Point(69.178, 109.814, Point.CARTESIAN),
                new Point(59.000, 94.011, Point.CARTESIAN)
        ));
        path.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90));
        return path;
    }
}
