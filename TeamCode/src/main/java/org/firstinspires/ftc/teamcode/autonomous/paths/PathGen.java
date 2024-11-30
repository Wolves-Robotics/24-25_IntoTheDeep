package org.firstinspires.ftc.teamcode.autonomous.paths;

import org.firstinspires.ftc.teamcode.autonomous.collections.StartPos;
import org.firstinspires.ftc.teamcode.autonomous.paths.parking.samplePark;
import org.firstinspires.ftc.teamcode.autonomous.paths.samples.firstSample.sample1ToBucket;
import org.firstinspires.ftc.teamcode.autonomous.paths.samples.firstSample.toSample1;
import org.firstinspires.ftc.teamcode.autonomous.paths.samples.secondSample.sample2ToBucket;
import org.firstinspires.ftc.teamcode.autonomous.paths.samples.secondSample.toSample2;
import org.firstinspires.ftc.teamcode.autonomous.paths.samples.thirdSample.sample3ToBucket;
import org.firstinspires.ftc.teamcode.autonomous.paths.samples.thirdSample.toSample3;
import org.firstinspires.ftc.teamcode.autonomous.paths.specimen.firstSpecimen.toPlace;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Path;

public class PathGen {
    private Path
            firstSpecimenPath,
            toSample1Path, sample1ToBucketPath,
            toSample2Path, sample2ToBucketPath,
            toSample3Path, sample3ToBucketPath,
            sampleParkPath;

    public void generate(StartPos startPos) {
        firstSpecimenPath = toPlace.getPath(startPos);

        toSample1Path = toSample1.getPath();
        sample1ToBucketPath = sample1ToBucket.getPath();
        toSample2Path = toSample2.getPath();
        sample2ToBucketPath = sample2ToBucket.getPath();
        toSample3Path = toSample3.getPath();
        sample3ToBucketPath = sample3ToBucket.getPath();

        sampleParkPath = samplePark.getPath();
    }

    public Path getFirstSpecimenPath() {return firstSpecimenPath;}

    public Path getToSample1Path() {return toSample1Path;}
    public Path getSample1ToBucketPath() {return sample1ToBucketPath;}
    public Path getToSample2Path() {return toSample2Path;}
    public Path getSample2ToBucketPath() {return sample2ToBucketPath;}
    public Path getToSample3Path() {return toSample3Path;}
    public Path getSample3ToBucketPath() {return  sample3ToBucketPath;}

    public Path getSampleParkPath() {return sampleParkPath;}
}
