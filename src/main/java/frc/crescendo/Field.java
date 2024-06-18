// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.crescendo;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in Meters</b> <br>
 * <br>
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class Field {

    // Angles should be red launch angles in degrees
    public static final double ampAimAngle = -39;
    public static final double podiumAimAngle = 26;
    public static final double ampWingAimAngle = 0;

    public static final double fieldLength = Units.inchesToMeters(651.223);
    public static final double fieldWidth = Units.inchesToMeters(323.277);
    public static final double wingX = Units.inchesToMeters(229.201);
    public static final double podiumX = Units.inchesToMeters(126.75);
    public static final double startingLineX = Units.inchesToMeters(74.111);

    public static final Translation2d ampCenter =
            new Translation2d(Units.inchesToMeters(72.455), fieldWidth);

    /** Staging locations for each note */
    public static final class StagingLocations {
        public static final double centerlineX = fieldLength / 2.0;

        // need to update
        public static final double centerlineFirstY = Units.inchesToMeters(29.638);
        public static final double centerlineSeparationY = Units.inchesToMeters(66);
        public static final double spikeX = Units.inchesToMeters(114);
        // need
        public static final double spikeFirstY = Units.inchesToMeters(161.638);
        public static final double spikeSeparationY = Units.inchesToMeters(57);

        public static final Translation2d[] centerlineTranslations = new Translation2d[5];
        public static final Translation2d[] spikeTranslations = new Translation2d[3];

        static {
            for (int i = 0; i < centerlineTranslations.length; i++) {
                centerlineTranslations[i] =
                        new Translation2d(
                                centerlineX, centerlineFirstY + (i * centerlineSeparationY));
            }
        }

        static {
            for (int i = 0; i < spikeTranslations.length; i++) {
                spikeTranslations[i] =
                        new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
            }
        }
    }

    /** Each corner of the speaker * */
    public static final class Speaker {

        // corners (blue alliance origin)
        public static final Translation3d topRightSpeaker =
                new Translation3d(
                        Units.inchesToMeters(18.055),
                        Units.inchesToMeters(238.815),
                        Units.inchesToMeters(83.091));

        public static final Translation3d topLeftSpeaker =
                new Translation3d(
                        Units.inchesToMeters(18.055),
                        Units.inchesToMeters(197.765),
                        Units.inchesToMeters(83.091));

        public static final Translation3d bottomRightSpeaker =
                new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
        public static final Translation3d bottomLeftSpeaker =
                new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));

        /** Center of the speaker opening (blue alliance) */
        public static final Translation3d centerSpeakerOpening =
                bottomLeftSpeaker.interpolate(topRightSpeaker, 0.5);

        public static final Pose2d centerSpeakerPose =
                new Pose2d(
                        centerSpeakerOpening.getX(), centerSpeakerOpening.getY(), new Rotation2d());
    }

    public static final class Subwoofer {
        public static final Pose2d ampFaceCorner =
                new Pose2d(
                        Units.inchesToMeters(35.775),
                        Units.inchesToMeters(239.366),
                        Rotation2d.fromDegrees(-120));

        public static final Pose2d sourceFaceCorner =
                new Pose2d(
                        Units.inchesToMeters(35.775),
                        Units.inchesToMeters(197.466),
                        Rotation2d.fromDegrees(120));

        public static final Pose2d centerFace =
                new Pose2d(
                        Units.inchesToMeters(35.775),
                        Units.inchesToMeters(218.416),
                        Rotation2d.fromDegrees(180));
    }

    public static final class Stage {
        public static final Pose2d podiumLeg =
                new Pose2d(
                        Units.inchesToMeters(126.75),
                        Units.inchesToMeters(161.638),
                        new Rotation2d());

        public static final Pose2d ampLeg =
                new Pose2d(
                        Units.inchesToMeters(220.873),
                        Units.inchesToMeters(212.425),
                        Rotation2d.fromDegrees(-30));

        public static final Pose2d sourceLeg =
                new Pose2d(
                        Units.inchesToMeters(220.873),
                        Units.inchesToMeters(110.837),
                        Rotation2d.fromDegrees(30));

        public static final Pose2d center =
                new Pose2d(
                        Units.inchesToMeters(192.55),
                        Units.inchesToMeters(161.638),
                        new Rotation2d());

        public static final Pose2d ampClimb =
                new Pose2d(12.265, 5.042, Rotation2d.fromDegrees(Field.flipAngleIfBlue(60)));

        public static final Pose2d centerClimb =
                new Pose2d(
                        ampLeg.getX(),
                        center.getY(),
                        Rotation2d.fromDegrees(Field.flipAngleIfBlue(180)));
    }

    public static final double aprilTagWidth = Units.inchesToMeters(6.50);

    /** Returns {@code true} if the robot is on the blue alliance. */
    public static boolean isBlue() {
        return DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue)
                .equals(DriverStation.Alliance.Blue);
    }

    /** Returns {@code true} if the robot is on the red alliance. */
    public static boolean isRed() {
        return !isBlue();
    }

    // Flip the angle if we are blue, as we are setting things for a red driver station angle
    public static double flipAngleIfBlue(double redAngleDegs) {
        if (Field.isBlue()) {
            return 180 - redAngleDegs;
        }
        return redAngleDegs;
    }

    public static Rotation2d flipAngleIfRed(Rotation2d blue) {
        if (Field.isRed()) {
            return new Rotation2d(-blue.getCos(), blue.getSin());
        } else {
            return blue;
        }
    }

    public static Pose2d flipXifRed(Pose2d blue) {
        return new Pose2d(
                flipXifRed(blue.getX()), blue.getTranslation().getY(), blue.getRotation());
    }

    public static Translation2d flipXifRed(Translation2d blue) {
        return new Translation2d(flipXifRed(blue.getX()), blue.getY());
    }

    public static Translation3d flipXifRed(Translation3d blue) {
        return new Translation3d(flipXifRed(blue.getX()), blue.getY(), blue.getZ());
    }

    // If we are red flip the x pose to the other side of the field
    public static double flipXifRed(double xCoordinate) {
        if (Field.isRed()) {
            return Field.fieldLength - xCoordinate;
        }
        return xCoordinate;
    }

    public static boolean poseOutOfField(Pose2d pose2D) {
        double x = pose2D.getX();
        double y = pose2D.getY();
        return (x <= 0 || x >= fieldLength) || (y <= 0 || y >= fieldWidth);
    }

    public static boolean poseOutOfField(Pose3d pose3D) {
        return poseOutOfField(pose3D.toPose2d());
    }

    // public static final AprilTagFieldLayout aprilTags;

    // static {
    //   aprilTags = Constants.aprilTagType.getLayoutSupplier().get();
    // }
}
