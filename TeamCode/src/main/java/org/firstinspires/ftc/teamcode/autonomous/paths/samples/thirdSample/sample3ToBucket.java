package org.firstinspires.ftc.teamcode.autonomous.paths.samples.thirdSample;

import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Point;

public class sample3ToBucket {
    public static Path getPath() {
        Path path = new Path(new BezierLine(
                new Point(33.500, 128.000, Point.CARTESIAN),
                new Point(14.100, 129.600, Point.CARTESIAN)
        ));
        path.setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(-45));
        return path;
    }
}
