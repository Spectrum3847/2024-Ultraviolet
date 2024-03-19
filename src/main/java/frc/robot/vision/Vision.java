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
import edu.wpi.first.wpilibj.DriverStation;
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
        public static double VISION_STD_DEV_X = 0.5;
        public static double VISION_STD_DEV_Y = 0.5;
        public static double VISION_STD_DEV_THETA = 99999999;

        public static final Matrix<N3, N1> visionStdMatrix =
                VecBuilder.fill(VISION_STD_DEV_X, VISION_STD_DEV_Y, VISION_STD_DEV_THETA);
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
    public final Limelight[] limelights = {speakerLL, rearLL};

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
            // Will NOT run in auto
            if (DriverStation.isTeleopEnabled()) {
                // force pose to be vision
                Pose2d estimatedPose = Robot.swerve.getPose();
                if ((estimatedPose.getX() <= 0.1 || estimatedPose.getY() <= 0.1)) {
                    forcePoseToVision();
                }

                isPresent = true;
                filterAndAddVisionMeasurment(speakerLL);
                filterAndAddVisionMeasurment(rearLL);

                // RobotTelemetry.print("added vision measurement");
            }
        } catch (NoSuchElementException e) {
            RobotTelemetry.print("Vision pose not present but tried to access it");
        }
    }

    private void filterAndAddVisionMeasurment(Limelight ll) {

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
            /* rejections */
            if (Field.poseOutOfField(botpose3D)) {
                // reject if pose is out of the field
                isPresent = false;
                ll.logStatus = "bound rejection";
                return;
            } else if (Math.abs(botpose3D.getZ()) > 0.25) {
                // reject if pose is .25 meters in the air
                isPresent = false;
                ll.logStatus = "height rejection";
                return;
            } else if (Math.abs(botpose3D.getRotation().getX()) > 5
                    || Math.abs(botpose3D.getRotation().getY()) > 5) {
                // reject if pose is 5 degrees titled in roll or pitch
                isPresent = false;
                ll.logStatus = "roll/pitch rejection";
            } else if (poseDifference < Units.inchesToMeters(3)) {
                // reject if pose is very close to robot pose
                isPresent = false;
                ll.logStatus = "proximity rejection";
                return;
            }
            /* integrations */
            else if (ll.multipleTagsInView() && targetSize > 0.05) {
                ll.logStatus = "Multi";
                xyStds = 0.5;
                degStds = 6;
            } else if (targetSize > 0.8 && poseDifference < 0.5) {
                ll.logStatus = "Close";
                xyStds = 1.0;
                degStds = 12;
            } else if (targetSize > 0.1 && poseDifference < 0.3) {
                ll.logStatus = "Proximity";
                xyStds = 2.0;
                degStds = 999999;
            } else {
                isPresent = false;
                ll.logStatus = "catch rejection";
                return;
            }

            ll.logStatus +=
                    " Pose integrated; New Odometry: "
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
            ll.logStatus = "target rejection";
            isPresent = false;
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

    public double getAdjustedThetaToSpeaker() {
        Translation2d speaker = getAdjustedSpeakerPos();
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

    public double getAdjustedThetaToFeeder() {
        Translation2d feeder = getAdjustedFeederPos();
        Translation2d robotXY = Robot.swerve.getPose().getTranslation();
        double angleBetweenRobotAndFeeder =
                MathUtil.angleModulus(feeder.minus(robotXY).getAngle().getRadians());

        return angleBetweenRobotAndFeeder;
    }

    /** Returns the distance from the feed position in meters, adjusted for the robot's movement. */
    @AutoLogOutput(key = "Vision/FeedDistance")
    public double getFeedDistance() {
        return Robot.swerve.getPose().getTranslation().getDistance(getAdjustedFeederPos());
    }

    public Translation2d getAdjustedFeederPos() {
        Translation2d originalLocation = Field.StagingLocations.spikeTranslations[2];
        return getAdjustedTargetPos(
                new Translation2d(originalLocation.getX() - 0.5, originalLocation.getY() - 0.5));
    }

    /** Set robot pose to vision pose regardless of validity. Does not reset rotation. */
    public void forcePoseToVision() {
        // TODO: add more fallback logic here
        Robot.swerve.resetPose(
                Robot.swerve.convertPoseWithGyro(speakerLL.getRawPose3d().toPose2d()));
    }

    /** Set robot pose to vision pose only if LL has good tag reading */
    public void resetPoseToVision() {
        for (Limelight limelight : limelights) {
            if (limelight.hasAccuratePose()) {
                Robot.swerve.resetPose(limelight.getRawPose3d().toPose2d());
                break;
            }
        }
    }

    /**
     * If at least one LL has an accurate pose
     *
     * @return
     */
    public boolean hasAccuratePose() {
        for (Limelight limelight : limelights) {
            if (limelight.hasAccuratePose()) return true;
        }
        return false;
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
        return speakerLL.isCameraConnected();
    }

    @AutoLogOutput(key = "Vision/Rear/ConnectionStatus")
    public boolean getRearConnection() {
        return rearLL.isCameraConnected();
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

    @AutoLogOutput(key = "Vision/Trust/STDTheta")
    public double getThetaSTD() {
        return VisionConfig.VISION_STD_DEV_THETA;
    }

    // @AutoLogOutput(key = "Vision/Front/TrustStrong")
    // public boolean getFrontTrustStrength() {
    //     return speakerLL.trustStrong;
    // }

    // @AutoLogOutput(key = "Vision/Rear/TrustStrong")
    // public boolean getRearTrustStrength() {
    //     return rearLL.trustStrong;
    // }

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
