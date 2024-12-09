package org.firstinspires.ftc.teamcode.autonomous.paths.specimen.firstSpecimen;

import org.firstinspires.ftc.teamcode.autonomous.collections.StartPos;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Point;

public class toPlace {
    public static Path getPath(StartPos startPos) {
        Path path = new Path(
            new BezierLine(
                new Point(7.000, startPos.equals(StartPos.specimen) ? 80:64, Point.CARTESIAN),
                new Point(39.000, startPos.equals(StartPos.specimen) ? 82:62, Point.CARTESIAN)
            )
        );
        path.setConstantHeadingInterpolation(Math.toRadians(180));
        return path;
    }
}
