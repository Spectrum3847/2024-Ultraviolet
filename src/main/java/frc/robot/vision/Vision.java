package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.crescendo.FieldConstants;
import frc.robot.Robot;
import frc.robot.RobotTelemetry;
import frc.spectrumLib.vision.Limelight;
import frc.spectrumLib.vision.Limelight.PhysicalConfig;
import java.text.DecimalFormat;
import java.util.NoSuchElementException;
import java.util.Optional;

public class Vision extends SubsystemBase {
    public static final class VisionConfig {
        /* Limelight Configuration */
        public static final String NOTE_LL =
                "limelight-detect"; // TODO: change this name in LL dashboard to reflect name in
        // code
        public static final PhysicalConfig NOTE_CONFIG =
                new PhysicalConfig().withTranslation(0, 0, 0).withRotation(0, 0, 0);

        public static final String SPEAKER_LL =
                "limelight-aim"; // TODO: change this name in LL dashboard to reflect name in code
        public static final PhysicalConfig SPEAKER_CONFIG =
                new PhysicalConfig().withTranslation(-0.051, 0, 0.69).withRotation(0, 0, 0);

        /* Pipeline config */
        public static final int noteDetectorPipeline = 0;
        public static final int speakerDetectorPipeline = 0;

        /* AprilTag Heights (meters) */
        public static final double speakerTagHeight = 1.45;
        public static final int speakerTagID = 4;

        /* Pose Estimation Constants */
        public static final double VISION_REJECT_DISTANCE = 1.8; // 2.3;
        // Increase these numbers to trust global measurements from vision less.
        // @AutoLogOutput(key = "Vision/STD/X")
        public static double VISION_STD_DEV_X = 0.5;

        // @AutoLogOutput(key = "Vision/STD/Y")
        public static double VISION_STD_DEV_Y = 0.5;

        public static double VISION_STD_DEV_THETA = 99999999;

        public static final Matrix<N3, N1> visionStdMatrix =
                VecBuilder.fill(VISION_STD_DEV_X, VISION_STD_DEV_Y, VISION_STD_DEV_THETA);

        /* Vision Command Configs */
        public static final class AlignToNote extends CommandConfig {
            private AlignToNote() {
                configKp(0.04);
                configTolerance(0.01);
                configMaxOutput(Robot.swerve.config.maxVelocity * 0.5);
                configError(0.3);
                configPipelineIndex(noteDetectorPipeline);
                configLimelight(Robot.vision.noteLL);
            }

            public static AlignToNote getConfig() {
                return new AlignToNote();
            }
        }

        public static final class AlignToSpeaker extends CommandConfig {
            private AlignToSpeaker() {
                configKp(0.04);
                configTolerance(0.01);
                configMaxOutput(Robot.swerve.config.maxVelocity * 0.5);
                configError(0.3);
                configPipelineIndex(speakerDetectorPipeline);
                configLimelight(Robot.vision.speakerLL);
            }

            public static AlignToSpeaker getConfig() {
                return new AlignToSpeaker();
            }
        }

        public static final class AlignToAmp extends CommandConfig {
            private AlignToAmp() {
                configKp(0.4);
                configTolerance(0);
                configMaxOutput(Robot.swerve.config.maxVelocity * 0.5);
                configError(0.3);
                configPipelineIndex(speakerDetectorPipeline);
                configLimelight(Robot.vision.speakerLL);
            }

            public static AlignToAmp getConfig() {
                return new AlignToAmp();
            }
        }

        public static final class AlignToStage extends CommandConfig {
            private AlignToStage() {
                configKp(0.04);
                configTolerance(0.01);
                configMaxOutput(Robot.swerve.config.maxVelocity * 0.5);
                configError(0.3);
                configPipelineIndex(speakerDetectorPipeline);
                configLimelight(Robot.vision.noteLL);
            }

            public static AlignToStage getConfig() {
                return new AlignToStage();
            }
        }

        public static final class DriveToNote extends CommandConfig {
            private DriveToNote() {
                configKp(0.3);
                configTolerance(0.05);
                configMaxOutput(Robot.swerve.config.maxVelocity * 0.5);
                configVerticalSetpoint(-8);
                configVerticalMaxView(6);
                configLimelight(Robot.vision.noteLL);
                configAlignCommand(AlignToNote.getConfig());
            }

            public static DriveToNote getConfig() {
                return new DriveToNote();
            }
        }
    }

    /* Limelights */
    public final Limelight noteLL =
            new Limelight(
                    VisionConfig.NOTE_LL,
                    VisionConfig.noteDetectorPipeline,
                    VisionConfig.NOTE_CONFIG);
    public final Limelight speakerLL =
            new Limelight(
                    VisionConfig.SPEAKER_LL,
                    VisionConfig.speakerDetectorPipeline,
                    VisionConfig.SPEAKER_CONFIG);
    public final Limelight[] limelights = {noteLL, speakerLL};

    private final DecimalFormat df = new DecimalFormat();

    /* Interpolator */
    public InterpolatingTreeMap<Double, Double> treeMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

    // @AutoLogOutput(key = "Vision/integratingPose")
    public static boolean isPresent = false;

    public Vision() {
        setName("Vision");
        addTreeMapData();

        // SmartDashboard.putBoolean("VisionPresent", Vision.isPresent);

        // logging
        df.setMaximumFractionDigits(2);

        /* Configure Limelight Settings Here */
        speakerLL.setLEDMode(false);
    }

    @Override
    public void periodic() {
        // Integrate Vision with Odometry only in teleop
        if (DriverStation.isTeleopEnabled()) {
            for (Limelight limelight : limelights) {
                try {
                    // retrieve pose if valid
                    Optional<Pose2d> visionPose =
                            limelight.getFilteredPose(
                                    Robot.swerve.getPose(), VisionConfig.VISION_REJECT_DISTANCE);
                    if (visionPose.isPresent()) {
                        isPresent = true;

                        // replace vision rotation component
                        Pose2d botpose = visionPose.get();
                        Pose2d poseWithGyro = Robot.swerve.convertPoseWithGyro(botpose);

                        // adjust vision trust
                        adjustVisionSTDs(limelight);
                        Matrix<N3, N1> visionSTDs =
                                VecBuilder.fill(
                                        VisionConfig.VISION_STD_DEV_X,
                                        VisionConfig.VISION_STD_DEV_Y,
                                        VisionConfig.VISION_STD_DEV_THETA);
                        Robot.swerve.setVisionMeasurementStdDevs(visionSTDs);

                        // integrate vision
                        Robot.swerve.addVisionMeasurement(
                                poseWithGyro, limelight.getVisionPoseTimestamp());
                        limelight.logStatus =
                                "Pose integrated; New Odometry: "
                                        + df.format(Robot.swerve.getPose().getX())
                                        + ", "
                                        + df.format(Robot.swerve.getPose().getY())
                                        + " || Vision Pose: "
                                        + df.format(poseWithGyro.getX())
                                        + ", "
                                        + df.format(poseWithGyro.getY());
                    } else {
                        isPresent = false;
                    }
                } catch (NoSuchElementException e) {
                    limelight.logStatus = "Optional Error while retrieving pose";
                    RobotTelemetry.print("Vision pose not present but tried to access it");
                }
            }
        }
    }

    // @AutoLogOutput(key = "Vision/Front/ConnectionStatus")
    // public boolean getFrontConnection() {
    //     return !NetworkTableInstance.getDefault()
    //             .getTable("limelight-aim")
    //             .getEntry("json")
    //             .getString("")
    //             .equals("");
    // }

    // @AutoLogOutput(key = "Vision/Rear/ConnectionStatus")
    // public boolean getRearConnection() {
    //     return !NetworkTableInstance.getDefault()
    //             .getTable("limelight-detect")
    //             .getEntry("json")
    //             .getString("")
    //             .equals("");
    // }

    // @AutoLogOutput(key = "Vision/Front/LogStatus")
    // public String getFrontLogStatus() {
    //     return speakerLL.logStatus;
    // }

    // @AutoLogOutput(key = "Vision/Rear/LogStatus")
    // public String getRearLogStatus() {
    //     return noteLL.logStatus;
    // }

    // @AutoLogOutput(key = "Vision/Front/TagCount")
    // public double getFrontTagCount() {
    //     return speakerLL.getTagCountInView();
    // }

    // @AutoLogOutput(key = "Vision/Rear/TagCount")
    // public double getRearTagCount() {
    //     return noteLL.getTagCountInView();
    // }

    // @AutoLogOutput(key = "Vision/Trust/STDX")
    // public double getXSTD() {
    //     return VisionConfig.VISION_STD_DEV_X;
    // }

    // @AutoLogOutput(key = "Vision/Trust/STDY")
    // public double getYSTD() {
    //     return VisionConfig.VISION_STD_DEV_Y;
    // }

    // @AutoLogOutput(key = "Vision/Front/TrustStrong")
    // public boolean getFrontTrustStrength() {
    //     return speakerLL.trustStrong;
    // }

    // @AutoLogOutput(key = "Vision/Rear/TrustStrong")
    // public boolean getRearTrustStrength() {
    //     return noteLL.trustStrong;
    // }

    /**
     * REQUIRES ACCURATE POSE ESTIMATION. Uses trigonometric functions to calculate the angle
     * between the robot heading and the angle required to face the hybrid spot. Will return 0 if
     * the robot cannot see an apriltag.
     *
     * @param hybridSpot 0-8 representing the 9 different hybrid spots for launching cubes to hybrid
     *     nodes
     * @return angle between robot heading and hybrid spot in degrees
     */
    public double getThetaToSpeaker() {
        Translation2d redSpeaker =
                new Translation2d(
                        FieldConstants.fieldLength
                                - FieldConstants.Speaker.centerSpeakerOpening
                                        .toTranslation2d()
                                        .getX(),
                        FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d().getY());
        double angleBetweenRobotAndSpeakers =
                redSpeaker.minus(Robot.swerve.getPose().getTranslation()).getAngle().getRadians();

        return angleBetweenRobotAndSpeakers;
    }

    public void adjustVisionSTDs(Limelight limelight) {
        double distanceToTag = limelight.getDistanceToTagFromCamera();
        double tagsInView = limelight.getTagCountInView();
        double trust = 1.2;
        if (tagsInView == 1) {
            trust = 100;
            if (distanceToTag <= 1.3) {
                trust = 0.1;
            }
        } else if (tagsInView == 2) {
            trust = 50;
            if (distanceToTag <= 1.5) {
                trust = 0.01;
            } else if (distanceToTag <= 3) {
                trust = 0.5;
            }
        } else if (tagsInView == 0) {
            trust = 200;
        }

        if (trust <= 1) {
            limelight.trustStrong = true;
        } else {
            limelight.trustStrong = false;
        }
        VisionConfig.VISION_STD_DEV_X = trust;
        VisionConfig.VISION_STD_DEV_Y = trust;
    }

    // we are resetting gyro angle as well?
    public void resetPoseWithVision() {
        // TODO: add more fallback logic here
        Robot.swerve.resetPose(
                Robot.swerve.convertPoseWithGyro(speakerLL.getRawPose3d().toPose2d()));
    }
    /**
     * Calculates the required rotation for the robot to align with a note, based on the current
     * orientation of the robot and the note's positional offset from the camera's center. The
     * result is the angle in degrees that the robot needs to turn to face the note directly.
     *
     * @return The angle in degrees to rotate the robot towards the note.
     */
    public double getOffsetToNote() {
        return Robot.swerve.getRotation().getDegrees() + noteLL.getHorizontalOffset();
    }

    /**
     * Calculates the required rotation for the robot to align with the speaker, based on the
     * current orientation of the robot and the speaker's positional offset from the camera's
     * center. The result is the angle in degrees that the robot needs to turn to face the speaker
     * directly.
     *
     * @return The angle in degrees to rotate the robot towards the speaker.
     */
    public double getOffsetToSpeaker() {
        return Robot.swerve.getRotation().getDegrees()
                - ((speakerLL.getClosestTagID() == VisionConfig.speakerTagID)
                        ? speakerLL.getHorizontalOffset()
                        : 0);
    }

    public double getDistanceToSpeaker() {
        return speakerLL.getDistanceToTarget(VisionConfig.speakerTagHeight);
    }

    public boolean noteInView() {
        return noteLL.targetInView();
    }

    public boolean speakerInView() {
        return speakerLL.targetInView();
    }

    /** Change both LL pipelines to the same pipeline */
    public void setLimelightPipelines(int pipeline) {
        noteLL.setLimelightPipeline(pipeline);
        speakerLL.setLimelightPipeline(pipeline);
    }

    /** Set both LLs to blink */
    public Command blinkLimelights() {
        return startEnd(
                        () -> {
                            noteLL.blinkLEDs();
                            speakerLL.blinkLEDs();
                        },
                        () -> {
                            noteLL.setLEDMode(false);
                            speakerLL.setLEDMode(false);
                        })
                .withName("Vision.blinkLimelights");
    }

    /**
     * Interpolates a pivot angle from the treemap based on how far the robot is away from the
     * speaker.
     *
     * @return
     */
    public double getAngleFromMap() {
        return treeMap.get(
                Robot.swerve.getPose().getY()); // TODO: change this to be relative from the speaker
    }

    /** Tree Map Data: key - distance (meters), value - pivot angle (degrees) */
    public void addTreeMapData() {
        treeMap.put(0.0, 0.0);
    }

    public static class CommandConfig {
        public double kp;
        public double tolerance;
        public double maxOutput;
        public double error;
        public int pipelineIndex;
        public Limelight limelight;
        /* For Drive-To commands */
        public CommandConfig alignCommand;
        public double verticalSetpoint; // numbers get small as the cone gets closer
        public double verticalMaxView;

        public void configKp(double kp) {
            this.kp = kp;
        }

        public void configTolerance(double tolerance) {
            this.tolerance = tolerance;
        }

        public void configMaxOutput(double maxOutput) {
            this.maxOutput = maxOutput;
        }

        public void configError(double error) {
            this.error = error;
        }

        public void configPipelineIndex(int pipelineIndex) {
            this.pipelineIndex = pipelineIndex;
        }

        public void configLimelight(Limelight limelight) {
            this.limelight = limelight;
        }

        public void configVerticalSetpoint(double verticalSetpoint) {
            this.verticalSetpoint = verticalSetpoint;
        }

        public void configVerticalMaxView(double verticalMaxView) {
            this.verticalMaxView = verticalMaxView;
        }

        public void configAlignCommand(CommandConfig alignCommand) {
            this.alignCommand = alignCommand;
        }

        public CommandConfig() {}
    }
}
