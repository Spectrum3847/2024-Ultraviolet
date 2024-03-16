package frc.robot.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.crescendo.Field;
import frc.robot.Robot;
import frc.robot.RobotTelemetry;
import frc.spectrumLib.vision.Limelight;
import frc.spectrumLib.vision.Limelight.PhysicalConfig;
import java.text.DecimalFormat;
import java.util.NoSuchElementException;
import org.littletonrobotics.junction.AutoLogOutput;

public class Vision extends SubsystemBase {
    public static final class VisionConfig {
        /* Limelight Configuration */
        public static final String REAR_LL = "limelight-rear";
        // These don't seem to actually be setting at the limelight, had to manually adjust them
        public static final PhysicalConfig REAR_CONFIG =
                new PhysicalConfig().withTranslation(-0.296, 0, 0.226).withRotation(0, 50, 180);

        public static final String SPEAKER_LL = "limelight-aim";

        public static final PhysicalConfig SPEAKER_CONFIG =
                new PhysicalConfig().withTranslation(-0.085, 0, 0.636).withRotation(0, 15, 0);

        // Limelight 2+
        // public static final PhysicalConfig SPEAKER_CONFIG =
        //         new PhysicalConfig().withTranslation(-0.085, 0, 0.636).withRotation(0, 15, 0);

        /* Pipeline config */
        public static final int rearDetectorPipeline = 0;
        public static final int speakerDetectorPipeline = 0;

        /* AprilTag Heights (meters) */
        public static final double speakerTagHeight = 1.45;
        public static final int speakerTagID = 4;

        /* Pose Estimation Constants */
        public static final double VISION_REJECT_DISTANCE = 1.8; // 2.3;
        // Increase these numbers to trust global measurements from vision less.
        @AutoLogOutput(key = "Vision/STD/X")
        public static double VISION_STD_DEV_X = 0.5;

        @AutoLogOutput(key = "Vision/STD/Y")
        public static double VISION_STD_DEV_Y = 0.5;

        public static double VISION_STD_DEV_THETA = 99999999;

        public static final Matrix<N3, N1> visionStdMatrix =
                VecBuilder.fill(VISION_STD_DEV_X, VISION_STD_DEV_Y, VISION_STD_DEV_THETA);

        /* Vision Command Configs */
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
                configLimelight(Robot.vision.rearLL);
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
                configLimelight(Robot.vision.rearLL);
            }

            public static AlignToStage getConfig() {
                return new AlignToStage();
            }
        }
    }

    /* Limelights */
    public final Limelight rearLL =
            new Limelight(
                    VisionConfig.REAR_LL,
                    VisionConfig.rearDetectorPipeline,
                    VisionConfig.REAR_CONFIG);
    public final Limelight speakerLL =
            new Limelight(
                    VisionConfig.SPEAKER_LL,
                    VisionConfig.speakerDetectorPipeline,
                    VisionConfig.SPEAKER_CONFIG);
    public final Limelight[] limelights = {rearLL, speakerLL};

    private final DecimalFormat df = new DecimalFormat();

    @AutoLogOutput(key = "Vision/integratingPose")
    public static boolean isPresent = false;

    public Vision() {
        setName("Vision");

        // SmartDashboard.putBoolean("VisionPresent", Vision.isPresent);

        // logging
        df.setMaximumFractionDigits(2);

        /* Configure Limelight Settings Here */
        speakerLL.setLEDMode(false);
        rearLL.setLEDMode(false);
    }

    @Override
    public void periodic() {
        try {
            // Will run in auto

            isPresent = true;
            // force pose to be vision
            Pose2d estimatedPose = Robot.swerve.getPose();
            if ((estimatedPose.getX() <= 0.1 || estimatedPose.getY() <= 0.1)) {
                resetPoseWithVision();
            }

            filterAndAddVisionMeasurment(speakerLL);
            filterAndAddVisionMeasurment(rearLL);

            // RobotTelemetry.print("added vision measurement");
        } catch (NoSuchElementException e) {
            RobotTelemetry.print("Vision pose not present but tried to access it");
        }
    }

    protected void filterAndAddVisionMeasurment(Limelight ll) {

        double xyStds = 1000;
        double degStds = 1000;

        // integrate vision
        if (ll.targetInView()) {
            Pose3d botpose3D = ll.getRawPose3d();
            double timeStamp = ll.getVisionPoseTimestamp();
            Pose2d botpose = botpose3D.toPose2d();

            // distance from current pose to vision estimated pose
            double poseDifference =
                    Robot.swerve.getPose().getTranslation().getDistance(botpose.getTranslation());

            double targetSize = ll.getTargetSize();
            if (Math.abs(botpose3D.getZ()) > 0.25
                    || poseDifference
                            < Units.inchesToMeters(
                                    3)) { // Reject if we think we are too high in the air
                isPresent = false;
                ll.logStatus = "rejected";
                return;
            } else if (ll.multipleTagsInView() && targetSize > 0.05) {
                xyStds = 0.5;
                degStds = 6;
            } else if (targetSize > 0.8 && poseDifference < 0.5) {
                xyStds = 1.0;
                degStds = 12;
            } else if (targetSize > 0.1 && poseDifference < 0.3) {
                xyStds = 2.0;
                degStds = 999999;
            } else {
                isPresent = false;
                // RobotTelemetry.print("Vision pose rejected");
                ll.logStatus = "rejected";
                return;
            }

            ll.logStatus =
                    "Pose integrated; New Odometry: "
                            + df.format(Robot.swerve.getPose().getX())
                            + ", "
                            + df.format(Robot.swerve.getPose().getY())
                            + " || Vision Pose: "
                            + df.format(botpose.getX())
                            + ", "
                            + df.format(botpose.getY());

            // track STDs
            VisionConfig.VISION_STD_DEV_X = xyStds;
            VisionConfig.VISION_STD_DEV_Y = xyStds;
            VisionConfig.VISION_STD_DEV_THETA = degStds;

            Robot.swerve.setVisionMeasurementStdDevs(
                    VecBuilder.fill(
                            VisionConfig.VISION_STD_DEV_X,
                            VisionConfig.VISION_STD_DEV_Y,
                            VisionConfig.VISION_STD_DEV_THETA));
            Robot.swerve.addVisionMeasurement(botpose, timeStamp);
        } else {
            return;
        }
    }

    /**
     * REQUIRES ACCURATE POSE ESTIMATION. Uses trigonometric functions to calculate the angle
     * between the robot heading and the angle required to face the speaker center.
     *
     * @return angle between robot heading and speaker in degrees
     */
    public double getThetaToSpeaker() {
        // Translation2d speaker =
        //         Field.flipXifRed(Field.Speaker.centerSpeakerOpening).toTranslation2d();
        Translation2d speaker =
                Field.flipXifRed(Field.Speaker.centerSpeakerPose)
                        .getTranslation(); // getAdjustedSpeakerPos();
        Translation2d robotXY = Robot.swerve.getPose().getTranslation();
        double angleBetweenRobotAndSpeaker =
                MathUtil.angleModulus(speaker.minus(robotXY).getAngle().getRadians());

        return angleBetweenRobotAndSpeaker;
    }

    /** Returns the distance from the speaker in meters, adjusted for the robot's movement. */
    @AutoLogOutput(key = "Vision/SpeakerDistance")
    public double getSpeakerDistance() {
        return Robot.swerve.getPose().getTranslation().getDistance(getAdjustedSpeakerPos());
    }

    public Translation2d getAdjustedSpeakerPos() {
        return getAdjustedTargetPos(
                new Translation2d(0, Field.Speaker.centerSpeakerOpening.toTranslation2d().getY()));
    }

    /**
     * Gets a field-relative position for the shot to the speaker the robot should take, adjusted
     * for the robot's movement.
     *
     * @return A {@link Translation2d} representing a field relative position in meters.
     */
    public Translation2d getAdjustedTargetPos(Translation2d targetPose) {
        double NORM_FUDGE = 0.075;
        double tunableNoteVelocity = 5.6;
        double tunableNormFudge = 0.85;
        double tunableStrafeFudge = 1.05;
        double tunableSpeakerYFudge = 0.0;
        double tunableSpeakerXFudge = 0.0;

        targetPose = Field.flipXifRed(targetPose);
        Translation2d robotPos = Robot.swerve.getPose().getTranslation();
        ChassisSpeeds robotVel = Robot.swerve.getVelocity(true); // TODO: change

        double distance = robotPos.getDistance(targetPose);
        double normFactor =
                Math.hypot(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond) < NORM_FUDGE
                        ? 0.0
                        : Math.abs(
                                MathUtil.angleModulus(
                                                robotPos.minus(targetPose).getAngle().getRadians()
                                                        - Math.atan2(
                                                                robotVel.vyMetersPerSecond,
                                                                robotVel.vxMetersPerSecond))
                                        / Math.PI);

        double x =
                targetPose.getX()
                        + (Field.isBlue() ? tunableSpeakerXFudge : -tunableSpeakerXFudge)
                        - (robotVel.vxMetersPerSecond
                                * (distance / tunableNoteVelocity)
                                * (1.0 - (tunableNormFudge * normFactor)));
        double y =
                targetPose.getY()
                        + tunableSpeakerYFudge
                        - (robotVel.vyMetersPerSecond
                                * (distance / tunableNoteVelocity)
                                * tunableStrafeFudge);

        return new Translation2d(x, y);
    }

    public double getAngleToFeederPos() {
        Translation2d feeder = getAdjustedFeederPos();
        Translation2d robotXY = Robot.swerve.getPose().getTranslation();
        double angleBetweenRobotAndFeeder =
                MathUtil.angleModulus(feeder.minus(robotXY).getAngle().getRadians());

        return angleBetweenRobotAndFeeder;
    }

    public Translation2d getAdjustedFeederPos() {
        return getAdjustedTargetPos(Field.StagingLocations.spikeTranslations[2]);
    }

    // public void beltonVision() {
    //     // Integrate Vision with Odometry only in teleop
    //     if (DriverStation.isTeleopEnabled()) {
    //         for (Limelight limelight : limelights) {
    //             try {
    //                 // retrieve pose if valid
    //                 Optional<Pose2d> visionPose =
    //                         limelight.getFilteredPose(
    //                                 Robot.swerve.getPose(), VisionConfig.VISION_REJECT_DISTANCE);
    //                 if (visionPose.isPresent()) {
    //                     isPresent = true;

    //                     // replace vision rotation component
    //                     Pose2d botpose = visionPose.get();
    //                     Pose2d poseWithGyro = Robot.swerve.convertPoseWithGyro(botpose);

    //                     // adjust vision trust
    //                     adjustVisionSTDs(limelight);
    //                     Matrix<N3, N1> visionSTDs =
    //                             VecBuilder.fill(
    //                                     VisionConfig.VISION_STD_DEV_X,
    //                                     VisionConfig.VISION_STD_DEV_Y,
    //                                     VisionConfig.VISION_STD_DEV_THETA);
    //                     Robot.swerve.setVisionMeasurementStdDevs(visionSTDs);

    //                     // integrate vision
    //                     Robot.swerve.addVisionMeasurement(
    //                             poseWithGyro, limelight.getVisionPoseTimestamp());
    //                     limelight.logStatus =
    //                             "Pose integrated; New Odometry: "
    //                                     + df.format(Robot.swerve.getPose().getX())
    //                                     + ", "
    //                                     + df.format(Robot.swerve.getPose().getY())
    //                                     + " || Vision Pose: "
    //                                     + df.format(poseWithGyro.getX())
    //                                     + ", "
    //                                     + df.format(poseWithGyro.getY());
    //                 } else {
    //                     isPresent = false;
    //                 }
    //             } catch (NoSuchElementException e) {
    //                 limelight.logStatus = "Optional Error while retrieving pose";
    //                 RobotTelemetry.print("Vision pose not present but tried to access it");
    //             }
    //         }
    //     }
    // }

    // public void beltonAdjustVisionSTDs(Limelight limelight) {
    //     double distanceToTag = limelight.getDistanceToTagFromCamera();
    //     double tagsInView = limelight.getTagCountInView();
    //     double trust = 1.2;
    //     if (tagsInView == 1) {
    //         trust = 100;
    //         if (distanceToTag <= 1.3) {
    //             trust = 0.1;
    //         }
    //     } else if (tagsInView == 2) {
    //         trust = 50;
    //         if (distanceToTag <= 1.5) {
    //             trust = 0.01;
    //         } else if (distanceToTag <= 3) {
    //             trust = 0.5;
    //         }
    //     } else if (tagsInView == 0) {
    //         trust = 200;
    //     }

    //     if (trust <= 1) {
    //         limelight.trustStrong = true;
    //     } else {
    //         limelight.trustStrong = false;
    //     }
    //     VisionConfig.VISION_STD_DEV_X = trust;
    //     VisionConfig.VISION_STD_DEV_Y = trust;
    // }

    // public Optional<Pose2d> getVisionPose(Limelight ll) {
    //     Pose2d visionPose = ll.getAlliancePose().toPose2d();

    //     // if no camera or no target in view, return empty
    //     if (!ll.isCameraConnected() || !ll.targetInView()) return Optional.empty();
    //     // if vision pose is too far off current, ignore it
    //     if (Robot.swerve.getPose().getTranslation().getDistance(visionPose.getTranslation())
    //                     < VisionConfig.VISION_REJECT_DISTANCE
    //             || (Robot.swerve.getPose().getX() <= 0 || Robot.swerve.getPose().getY() <= 0)) {
    //         return Optional.of(new Pose2d(visionPose.getTranslation(),
    // Robot.swerve.getRotation()));
    //     }

    //     return Optional.empty();
    // }

    // we are resetting gyro angle as well?
    public void resetPoseWithVision() {
        // TODO: add more fallback logic here
        Robot.swerve.resetPose(
                Robot.swerve.convertPoseWithGyro(speakerLL.getRawPose3d().toPose2d()));
    }

    /** Change both LL pipelines to the same pipeline */
    public void setLimelightPipelines(int pipeline) {
        rearLL.setLimelightPipeline(pipeline);
        speakerLL.setLimelightPipeline(pipeline);
    }

    /** Set both LLs to blink */
    public Command blinkLimelights() {
        return startEnd(
                        () -> {
                            rearLL.blinkLEDs();
                            speakerLL.blinkLEDs();
                        },
                        () -> {
                            rearLL.setLEDMode(false);
                            speakerLL.setLEDMode(false);
                        })
                .withName("Vision.blinkLimelights");
    }

    /** Logging */
    @AutoLogOutput(key = "Vision/Front/ConnectionStatus")
    public boolean getFrontConnection() {
        return !NetworkTableInstance.getDefault()
                .getTable("limelight-aim")
                .getEntry("json")
                .getString("")
                .equals("");
    }

    @AutoLogOutput(key = "Vision/Rear/ConnectionStatus")
    public boolean getRearConnection() {
        return !NetworkTableInstance.getDefault()
                .getTable("limelight-detect")
                .getEntry("json")
                .getString("")
                .equals("");
    }

    @AutoLogOutput(key = "Vision/Front/LogStatus")
    public String getFrontLogStatus() {
        return speakerLL.logStatus;
    }

    @AutoLogOutput(key = "Vision/Rear/LogStatus")
    public String getRearLogStatus() {
        return rearLL.logStatus;
    }

    @AutoLogOutput(key = "Vision/Front/TagCount")
    public double getFrontTagCount() {
        return speakerLL.getTagCountInView();
    }

    @AutoLogOutput(key = "Vision/Rear/TagCount")
    public double getRearTagCount() {
        return rearLL.getTagCountInView();
    }

    @AutoLogOutput(key = "Vision/Trust/STDX")
    public double getXSTD() {
        return VisionConfig.VISION_STD_DEV_X;
    }

    @AutoLogOutput(key = "Vision/Trust/STDY")
    public double getYSTD() {
        return VisionConfig.VISION_STD_DEV_Y;
    }

    @AutoLogOutput(key = "Vision/Front/TrustStrong")
    public boolean getFrontTrustStrength() {
        return speakerLL.trustStrong;
    }

    @AutoLogOutput(key = "Vision/Rear/TrustStrong")
    public boolean getRearTrustStrength() {
        return rearLL.trustStrong;
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
