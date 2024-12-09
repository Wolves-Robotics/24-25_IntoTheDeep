package org.firstinspires.ftc.teamcode.autonomous.paths.samples.secondSample;

import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Point;

public class toSample2 {
    public static Path getPath() {
        Path path = new Path(new BezierLine(
                new Point(14.190, 129.648, Point.CARTESIAN),
                new Point(27.091, 130.132, Point.CARTESIAN)
        ));
        path.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0));
        return path;
    }
}
