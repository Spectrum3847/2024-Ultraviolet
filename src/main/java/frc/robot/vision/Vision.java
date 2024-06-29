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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.crescendo.Field;
import frc.robot.Robot;
import frc.robot.RobotTelemetry;
import frc.robot.leds.LEDsCommands;
import frc.spectrumLib.util.Trio;
import frc.spectrumLib.vision.Limelight;
import frc.spectrumLib.vision.Limelight.PhysicalConfig;
import frc.spectrumLib.vision.LimelightHelpers.RawFiducial;
import java.text.DecimalFormat;
import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLogOutput;

public class Vision extends SubsystemBase {
    /**
     * Configs must be initialized and added as limelights to {@link Vision} {@code allLimelights} &
     * {@code poseLimelights}
     */
    public static final class VisionConfig {
        /* Limelight Configuration */

        public static final String FRONT_LL = "limelight-front";
        public static final PhysicalConfig FRONT_CONFIG =
                new PhysicalConfig().withTranslation(-0.085, 0, 0.636).withRotation(0, 15, 0);

        public static final String REAR_LL = "limelight-rear";
        public static final PhysicalConfig REAR_CONFIG =
                new PhysicalConfig()
                        .withTranslation(-0.287, -0.095, 0.37)
                        .withRotation(0, 15.744, 180);

        public static final String LEFT_LL = "limelight-left";
        public static final PhysicalConfig LEFT_CONFIG =
                new PhysicalConfig().withTranslation(-0.135, -0.315, 0.417).withRotation(0, 10, 90);

        public static final String RIGHT_LL = "limelight-right";
        public static final PhysicalConfig RIGHT_CONFIG =
                new PhysicalConfig().withTranslation(-0.135, 0.315, 0.417).withRotation(0, 10, -90);

        public static final String DETECT_LL = "limelight-detect";
        public static final PhysicalConfig DETECT_CONFIG =
                new PhysicalConfig().withTranslation(0, 0, 0).withRotation(0, 0, 0); // TODO: input

        /* Pipeline configs */
        public static final int frontTagPipeline = 0;
        public static final int rearTagPipeline = 0;
        public static final int leftTagPipeline = 0;
        public static final int rightTagPipeline = 0;
        public static final int detectNotePipeline = 0;

        /* Pose Estimation Constants */
        public static final double VISION_REJECT_DISTANCE = 1.8; // 2.3;

        // Increase these numbers to trust global measurements from vision less.
        public static double VISION_STD_DEV_X = 0.5;
        public static double VISION_STD_DEV_Y = 0.5;
        public static double VISION_STD_DEV_THETA = 99999999;

        public static final Matrix<N3, N1> visionStdMatrix =
                VecBuilder.fill(VISION_STD_DEV_X, VISION_STD_DEV_Y, VISION_STD_DEV_THETA);

        /* Vision Command Configs */
        public static final class AlignToNote extends CommandConfig {
            private AlignToNote() {
                configKp(0.02);
                configTolerance(0.01);
                configMaxOutput(Robot.swerve.config.maxVelocity * 0.5);
                configError(0.3);
                configPipelineIndex(detectNotePipeline);
                configLimelight(Robot.vision.detectLL);
            }

            public static AlignToNote getConfig() {
                return new AlignToNote();
            }
        }

        public static final class DriveToNote extends CommandConfig {
            private DriveToNote() {
                configKp(0.2);
                configTolerance(0.05);
                configMaxOutput(Robot.swerve.config.maxVelocity * 0.5);
                configVerticalSetpoint(-23);
                configVerticalMaxView(15);
                configLimelight(Robot.vision.detectLL);
                configAlignCommand(AlignToNote.getConfig());
            }

            public static DriveToNote getConfig() {
                return new DriveToNote();
            }
        }
    }

    /* Limelights */
    public final Limelight rearLL =
            new Limelight(
                    VisionConfig.REAR_LL, VisionConfig.rearTagPipeline, VisionConfig.REAR_CONFIG);
    public final LimelightLogger rearLogger = new LimelightLogger("Rear", rearLL);
    public final Limelight frontLL =
            new Limelight(
                    VisionConfig.FRONT_LL,
                    VisionConfig.frontTagPipeline,
                    VisionConfig.FRONT_CONFIG);
    public final LimelightLogger frontLogger = new LimelightLogger("Front", frontLL);
    public final Limelight leftLL =
            new Limelight(
                    VisionConfig.LEFT_LL, VisionConfig.leftTagPipeline, VisionConfig.LEFT_CONFIG);
    public final LimelightLogger leftLogger = new LimelightLogger("Left", leftLL);
    public final Limelight rightLL =
            new Limelight(
                    VisionConfig.RIGHT_LL,
                    VisionConfig.rightTagPipeline,
                    VisionConfig.RIGHT_CONFIG);
    public final LimelightLogger rightLogger = new LimelightLogger("Right", rightLL);
    public final Limelight detectLL =
            new Limelight(
                    VisionConfig.DETECT_LL,
                    VisionConfig.detectNotePipeline,
                    VisionConfig.DETECT_CONFIG);
    public final Limelight[] allLimelights = {frontLL, rearLL, leftLL, rightLL, detectLL};
    public final Limelight[] poseLimelights = {
        frontLL, rearLL, leftLL, rightLL
    }; // EXCLUDES DETECT LL

    private final DecimalFormat df = new DecimalFormat();

    // @AutoLogOutput(key = "Vision/a_Integrating")
    public static boolean isIntegrating = false;

    public ArrayList<Trio<Pose3d, Pose2d, Double>> autonPoses =
            new ArrayList<Trio<Pose3d, Pose2d, Double>>();

    private boolean isAiming = false;

    public Vision() {
        setName("Vision");

        // logging
        df.setMaximumFractionDigits(2);

        /* Configure Limelight Settings Here */
        for (Limelight limelight : allLimelights) {
            limelight.setLEDMode(false);
        }

        detectLL.setLEDMode(true);
    }

    @Override
    public void periodic() {
        double yaw = Robot.swerve.getRotation().getDegrees();
        for (Limelight limelight : poseLimelights) {
            limelight.setRobotOrientation(yaw);

            if (DriverStation.isAutonomousEnabled() && limelight.targetInView()) {
                Pose3d botpose3D = limelight.getRawPose3d();
                Pose2d megaPose2d = limelight.getMegaPose2d();
                double timeStamp = limelight.getRawPoseTimestamp();
                Pose2d integratablePose =
                        new Pose2d(megaPose2d.getTranslation(), botpose3D.toPose2d().getRotation());
                autonPoses.add(Trio.of(botpose3D, integratablePose, timeStamp));
            }
        }

        try {
            isIntegrating = false;
            // Will NOT run in auto
            if (DriverStation.isTeleopEnabled()) {
                // if the front camera sees tag and we are aiming, only use that camera
                // if (isAiming && speakerLL.targetInView()) {
                //     for (Limelight limelight : limelights) {
                //         if (limelight.CAMERA_NAME == speakerLL.CAMERA_NAME) {
                //             addFilteredVisionInput(limelight);
                //         } else {
                //             limelight.sendInvalidStatus("speaker only rejection");
                //         }
                //         isIntegrating |= limelight.isIntegrating;
                //     }
                // } else {
                // choose LL with best view of tags and integrate from only that camera
                Limelight bestLimelight = getBestLimelight();
                for (Limelight limelight : poseLimelights) {
                    if (limelight.CAMERA_NAME == bestLimelight.CAMERA_NAME) {
                        addFilteredVisionInput(bestLimelight);
                    } else {
                        limelight.sendInvalidStatus("not best rejection");
                    }
                    isIntegrating |= limelight.isIntegrating;
                }
                // }
            }
        } catch (Exception e) {
            RobotTelemetry.print("Vision pose not present but tried to access it");
        }
    }

    private void addFilteredVisionInput(Limelight ll) {
        double xyStds = 1000;
        double degStds = 1000;

        // integrate vision
        if (ll.targetInView()) {
            boolean multiTags = ll.multipleTagsInView();
            double timeStamp = ll.getRawPoseTimestamp();
            double targetSize = ll.getTargetSize();
            Pose3d botpose3D = ll.getRawPose3d();
            Pose2d botpose = botpose3D.toPose2d();
            Pose2d megaPose2d = ll.getMegaPose2d();
            RawFiducial[] tags = ll.getRawFiducial();
            double highestAmbiguity = 2;
            ChassisSpeeds robotSpeed = Robot.swerve.getVelocity(true);

            // distance from current pose to vision estimated pose
            double poseDifference =
                    Robot.swerve.getPose().getTranslation().getDistance(botpose.getTranslation());

            /* rejections */
            // reject pose if individual tag ambiguity is too high
            ll.tagStatus = "";
            for (RawFiducial tag : tags) {
                // search for highest ambiguity tag for later checks
                if (highestAmbiguity == 2) {
                    highestAmbiguity = tag.ambiguity;
                } else if (tag.ambiguity > highestAmbiguity) {
                    highestAmbiguity = tag.ambiguity;
                }
                // log ambiguities
                ll.tagStatus += "Tag " + tag.id + ": " + tag.ambiguity;
                // ambiguity rejection check
                if (tag.ambiguity > 0.9) {
                    ll.sendInvalidStatus("ambiguity rejection");
                    return;
                }
            }
            if (Field.poseOutOfField(botpose3D)) {
                // reject if pose is out of the field
                ll.sendInvalidStatus("bound rejection");
                return;
            } else if (Math.abs(robotSpeed.omegaRadiansPerSecond) >= 1.6) {
                // reject if we are rotating more than 0.5 rad/s
                ll.sendInvalidStatus("rotation rejection");
                return;
            } else if (Math.abs(botpose3D.getZ()) > 0.25) {
                // reject if pose is .25 meters in the air
                ll.sendInvalidStatus("height rejection");
                return;
            } else if (Math.abs(botpose3D.getRotation().getX()) > 5
                    || Math.abs(botpose3D.getRotation().getY()) > 5) {
                // reject if pose is 5 degrees titled in roll or pitch
                ll.sendInvalidStatus("roll/pitch rejection");
                return;
            } else if (targetSize <= 0.025) {
                ll.sendInvalidStatus("size rejection");
                return;
            }
            /* integrations */
            // if almost stationary and extremely close to tag
            else if (robotSpeed.vxMetersPerSecond + robotSpeed.vyMetersPerSecond <= 0.2
                    && targetSize > 0.45) { // 0.4
                ll.sendValidStatus("Stationary close integration");
                xyStds = 0.1;
                degStds = 0.1;
            } else if (multiTags && targetSize > 0.05) {
                ll.sendValidStatus("Multi integration");
                xyStds = 0.20; // 0.25
                degStds = 8;
                if (targetSize > 0.09) {
                    ll.sendValidStatus("Strong Multi integration");
                    xyStds = 0.1;
                    degStds = 0.1;
                }
            } else if (targetSize > 0.8 && poseDifference < 0.5) {
                ll.sendValidStatus("Close integration");
                xyStds = 0.5;
                degStds = 16;
            } else if (targetSize > 0.1 && poseDifference < 0.3) {
                ll.sendValidStatus("Proximity integration");
                xyStds = 2.0;
                degStds = 999999;
            } else if (highestAmbiguity < 0.25 && targetSize >= 0.03) {
                ll.sendValidStatus("Stable integration");
                xyStds = 0.5;
                degStds = 999999;
            } else {
                ll.sendInvalidStatus(
                        "catch rejection: "
                                + RobotTelemetry.truncatedDouble(poseDifference)
                                + " poseDiff");
                return;
            }

            // strict with degree std and ambiguity and rotation because this is megatag1
            if (highestAmbiguity > 0.5) {
                degStds = 15;
            }

            if (robotSpeed.omegaRadiansPerSecond >= 0.5) {
                degStds = 15;
            }

            // track STDs
            VisionConfig.VISION_STD_DEV_X = xyStds;
            VisionConfig.VISION_STD_DEV_Y = xyStds;
            VisionConfig.VISION_STD_DEV_THETA = degStds;

            Robot.swerve.setVisionMeasurementStdDevs(
                    VecBuilder.fill(
                            VisionConfig.VISION_STD_DEV_X,
                            VisionConfig.VISION_STD_DEV_Y,
                            VisionConfig.VISION_STD_DEV_THETA));

            Pose2d integratedPose = new Pose2d(megaPose2d.getTranslation(), botpose.getRotation());
            Robot.swerve.addVisionMeasurement(integratedPose, timeStamp);
        } else {
            ll.tagStatus = "no tags";
            ll.sendInvalidStatus("no tag found rejection");
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
    // @AutoLogOutput(key = "Vision/SpeakerDistance")
    public double getSpeakerDistance() {
        double poseDistance =
                Robot.swerve.getPose().getTranslation().getDistance(getAdjustedSpeakerPos());
        double tagDistance = getDistanceToCenterSpeakerTagFromRobot();
        if (tagDistance != -1) {
            return poseDistance; // tagDistance;
        }
        return poseDistance;
    }

    // @AutoLogOutput(key = "Vision/SpeakerYDistance")
    public double getSpeakerYDelta() {
        return Robot.swerve.getPose().getTranslation().getY() - getAdjustedSpeakerPos().getY();
    }

    public Translation2d getAdjustedSpeakerPos() {
        return getAdjustedTargetPos(
                new Translation2d(0, Field.Speaker.centerSpeakerOpening.toTranslation2d().getY()));
    }

    // Returns distance to the center of the speaker tag from the robot or -1 if not found
    public double getDistanceToCenterSpeakerTagFromRobot() {
        RawFiducial[] tags = frontLL.getRawFiducial();
        int speakerTagID = 7; // Blue Speaker Tag
        if (Field.isRed()) {
            speakerTagID = 4; // Red Speaker Tag
        }

        for (RawFiducial tag : tags) {
            if (tag.id == speakerTagID) {
                return tag.distToRobot;
            }
        }

        return -1;
    }

    /**
     * Gets a field-relative position for the shot to the speaker the robot should take, adjusted
     * for the robot's movement.
     *
     * @return A {@link Translation2d} representing a field relative position in meters.
     */
    public Translation2d getAdjustedTargetPos(Translation2d targetPose) {
        double NORM_FUDGE = 0.075;
        double tunableNoteVelocity = 1;
        double tunableNormFudge = 0;
        double tunableStrafeFudge = 1;
        double tunableSpeakerYFudge = 0.0;
        double tunableSpeakerXFudge = 0.0;

        Translation2d robotPos = Robot.swerve.getPose().getTranslation();
        targetPose = Field.flipXifRed(targetPose);
        double xDifference = Math.abs(robotPos.getX() - targetPose.getX());
        double spinYFudge =
                (xDifference < 5.8)
                        ? 0.05
                        : 0.8; // change spin fudge for score distances vs. feed distances

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
                targetPose.getX() + (Field.isBlue() ? tunableSpeakerXFudge : -tunableSpeakerXFudge);
        // - (robotVel.vxMetersPerSecond * (distance / tunableNoteVelocity));
        //      * (1.0 - (tunableNormFudge * normFactor)));
        double y =
                targetPose.getY()
                        + (Field.isBlue() ? -spinYFudge : spinYFudge)
                        + tunableSpeakerYFudge;
        // - (robotVel.vyMetersPerSecond * (distance / tunableNoteVelocity));
        //       * tunableStrafeFudge);

        return new Translation2d(x, y);
    }

    public double getAdjustedThetaToFeeder() {
        Translation2d feeder = getAdjustedFeederPos();
        Translation2d robotXY = Robot.swerve.getPose().getTranslation();
        double angleBetweenRobotAndFeeder =
                MathUtil.angleModulus(feeder.minus(robotXY).getAngle().getRadians());

        return angleBetweenRobotAndFeeder;
    }

    public double getAdjustedThetaToDeepFeeder() {
        Translation2d feeder = getAdjustedDeepFeederPos();
        Translation2d robotXY = Robot.swerve.getPose().getTranslation();
        double angleBetweenRobotAndDeepFeeder =
                MathUtil.angleModulus(feeder.minus(robotXY).getAngle().getRadians());

        return angleBetweenRobotAndDeepFeeder;
    }

    /** Returns the distance from the feed position in meters, adjusted for the robot's movement. */
    // @AutoLogOutput(key = "Vision/FeedDistance")
    public double getFeedDistance() {
        return Robot.swerve.getPose().getTranslation().getDistance(getAdjustedFeederPos());
    }

    /**
     * Returns the distance from the deep feed position in meters, adjusted for the robot's
     * movement.
     */
    // @AutoLogOutput(key = "Vision/DeepFeedDistance")
    public double getDeepFeedDistance() {
        return Robot.swerve.getPose().getTranslation().getDistance(getAdjustedDeepFeederPos());
    }

    public Translation2d getAdjustedFeederPos() {
        Translation2d originalLocation = Field.StagingLocations.spikeTranslations[1];
        Translation2d newLocation;
        if (Field.isBlue()) {
            newLocation =
                    new Translation2d(originalLocation.getX() - 0.5, originalLocation.getY() + 0.5);
        } else {
            newLocation = new Translation2d(originalLocation.getX(), originalLocation.getY());
        }
        return getAdjustedTargetPos(newLocation);
    }

    public Translation2d getAdjustedDeepFeederPos() {
        Translation2d originalLocation = Field.StagingLocations.spikeTranslations[1];
        Translation2d newLocation;
        if (Field.isBlue()) {
            newLocation = new Translation2d(originalLocation.getX(), originalLocation.getY());
        } else {
            newLocation = new Translation2d(originalLocation.getX(), originalLocation.getY());
        }
        return getAdjustedTargetPos(newLocation);
    }

    /**
     * Set robot pose to vision pose ONLY USING SPEAKER LL regardless of validity. Does not reset
     * rotation.
     */
    public void forcePoseToVision() {
        // TODO: add more fallback logic here
        Robot.swerve.resetPose(Robot.swerve.convertPoseWithGyro(frontLL.getRawPose3d().toPose2d()));
    }

    public void autonResetPoseToVision() {
        boolean reject = true;
        boolean firstSuccess = false;
        double batchSize = 5;
        for (int i = autonPoses.size() - 1; i > autonPoses.size() - (batchSize + 1); i--) {
            Trio<Pose3d, Pose2d, Double> poseInfo = autonPoses.get(i);
            boolean success =
                    resetPoseToVision(
                            true, poseInfo.getFirst(), poseInfo.getSecond(), poseInfo.getThird());
            if (success) {
                if (i == autonPoses.size() - 1) {
                    firstSuccess = true;
                }
                reject = false;
                RobotTelemetry.print(
                        "AutonResetPoseToVision succeeded on " + (autonPoses.size() - i) + " try");
                break;
            }
        }

        if (reject) {
            RobotTelemetry.print(
                    "AutonResetPoseToVision failed after "
                            + batchSize
                            + " of "
                            + autonPoses.size()
                            + " possible tries");
            LEDsCommands.solidErrorLED().withTimeout(0.8).schedule();
        } else {
            if (firstSuccess) {
                LEDsCommands.solidGreenLED().withTimeout(0.8).schedule();
            } else {
                LEDsCommands.solidOrangeLED().withTimeout(0.8).schedule();
            }
        }
    }

    public void resetPoseToVision() {
        Limelight ll = getBestLimelight();
        resetPoseToVision(
                ll.targetInView(), ll.getRawPose3d(), ll.getMegaPose2d(), ll.getRawPoseTimestamp());
    }

    /**
     * Set robot pose to vision pose only if LL has good tag reading
     *
     * @return if the pose was accepted and integrated
     */
    public boolean resetPoseToVision(
            boolean targetInView, Pose3d botpose3D, Pose2d megaPose, double poseTimestamp) {
        boolean reject = false;
        if (targetInView) {
            Pose2d botpose = botpose3D.toPose2d();
            Pose2d robotPose = Robot.swerve.getPose();
            if (Field.poseOutOfField(botpose3D)
                    || Math.abs(botpose3D.getZ()) > 0.25
                    || (Math.abs(botpose3D.getRotation().getX()) > 5
                            || Math.abs(botpose3D.getRotation().getY()) > 5)) {
                RobotTelemetry.print(
                        "ResetPoseToVision: FAIL || DID NOT RESET POSE TO VISION BECAUSE BAD POSE");
                reject = true;
            }
            if (Field.poseOutOfField(botpose3D)) {
                RobotTelemetry.print(
                        "ResetPoseToVision: FAIL || DID NOT RESET POSE TO VISION BECAUSE OUT OF FIELD");
                reject = true;
            } else if (Math.abs(botpose3D.getZ()) > 0.25) {
                RobotTelemetry.print(
                        "ResetPoseToVision: FAIL || DID NOT RESET POSE TO VISION BECAUSE IN AIR");
                reject = true;
            } else if ((Math.abs(botpose3D.getRotation().getX()) > 5
                    || Math.abs(botpose3D.getRotation().getY()) > 5)) {
                RobotTelemetry.print(
                        "ResetPoseToVision: FAIL || DID NOT RESET POSE TO VISION BECAUSE TILTED");
                reject = true;
            }

            // don't continue
            if (reject) {
                return !reject; // return the success status
            }

            // track STDs
            VisionConfig.VISION_STD_DEV_X = 0.001;
            VisionConfig.VISION_STD_DEV_Y = 0.001;
            VisionConfig.VISION_STD_DEV_THETA = 0.001;

            RobotTelemetry.print(
                    "ResetPoseToVision: Old Pose X: "
                            + RobotTelemetry.truncatedDouble(robotPose.getX())
                            + " Y: "
                            + RobotTelemetry.truncatedDouble(robotPose.getY())
                            + " Theta: "
                            + RobotTelemetry.truncatedDouble(robotPose.getRotation().getDegrees()));
            Robot.swerve.setVisionMeasurementStdDevs(
                    VecBuilder.fill(
                            VisionConfig.VISION_STD_DEV_X,
                            VisionConfig.VISION_STD_DEV_Y,
                            VisionConfig.VISION_STD_DEV_THETA));

            Pose2d integratedPose = new Pose2d(megaPose.getTranslation(), botpose.getRotation());
            Robot.swerve.addVisionMeasurement(integratedPose, poseTimestamp);
            robotPose = Robot.swerve.getPose(); // get updated pose
            RobotTelemetry.print(
                    "ResetPoseToVision: New Pose X: "
                            + RobotTelemetry.truncatedDouble(robotPose.getX())
                            + " Y: "
                            + RobotTelemetry.truncatedDouble(robotPose.getY())
                            + " Theta: "
                            + RobotTelemetry.truncatedDouble(robotPose.getRotation().getDegrees()));
            RobotTelemetry.print("ResetPoseToVision: SUCCESS");
            return true;
        }
        return false; // target not in view
    }

    public Limelight getBestLimelight() {
        Limelight bestLimelight = frontLL;
        double bestScore = 0;
        for (Limelight limelight : poseLimelights) {
            double score = 0;
            // prefer LL with most tags, when equal tag count, prefer LL closer to tags
            score += limelight.getTagCountInView();
            score += limelight.getTargetSize();

            if (score > bestScore) {
                bestScore = score;
                bestLimelight = limelight;
            }
        }
        return bestLimelight;
    }

    // @AutoLogOutput(key = "Vision/BestLimelight")
    public String logBestLimelight() {
        return getBestLimelight().CAMERA_NAME;
    }

    @AutoLogOutput(key = "Vision/NoteInView")
    public boolean noteInView() {
        return detectLL.targetInView();
    }

    /**
     * If at least one LL has an accurate pose
     *
     * @return
     */
    public boolean hasAccuratePose() {
        for (Limelight limelight : poseLimelights) {
            if (limelight.hasAccuratePose()) return true;
        }
        return false;
    }

    /** Change all LL pipelines to the same pipeline */
    public void setLimelightPipelines(int pipeline) {
        for (Limelight limelight : allLimelights) {
            limelight.setLimelightPipeline(pipeline);
        }
    }

    /** Set both LLs to blink */
    public Command blinkLimelights() {
        return startEnd(
                        () -> {
                            for (Limelight limelight : allLimelights) {
                                limelight.blinkLEDs();
                            }
                        },
                        () -> {
                            for (Limelight limelight : allLimelights) {
                                limelight.setLEDMode(false);
                            }
                        })
                .withName("Vision.blinkLimelights");
    }

    /** Set rear LL to blink */
    public Command solidLimelight() {
        return startEnd(
                        () -> {
                            rearLL.setLEDMode(true);
                        },
                        () -> {
                            rearLL.setLEDMode(false);
                        })
                .withName("Vision.blinkLimelights");
    }

    public void setAiming() {
        isAiming = true;
    }

    public void setNotAiming() {
        isAiming = false;
    }

    /** Logging */

    // can't use autologoutput in library and avoid repetitive loggers
    public static class LimelightLogger {
        private final Limelight limelight;
        private String name;

        public LimelightLogger(String name, Limelight limelight) {
            this.limelight = limelight;
            this.name = name;
        }

        // @AutoLogOutput(key = "Vision/{name}/ConnectionStatus")
        public boolean getCameraConnection() {
            return limelight.isCameraConnected();
        }

        // @AutoLogOutput(key = "Vision/{name}/Integrating")
        public boolean getIntegratingStatus() {
            return limelight.isIntegrating;
        }

        // @AutoLogOutput(key = "Vision/{name}/LogStatus")
        public String getLogStatus() {
            return limelight.logStatus;
        }

        // @AutoLogOutput(key = "Vision/{name}/TagStatus")
        public String getTagStatus() {
            return limelight.tagStatus;
        }

        // @AutoLogOutput(key = "Vision/{name}/Pose")
        public Pose2d getPose() {
            return limelight.getRawPose3d().toPose2d();
        }

        // @AutoLogOutput(key = "Vision/{name}/MegaPose")
        public Pose2d getMegaPose() {
            return limelight.getMegaPose2d();
        }

        // @AutoLogOutput(key = "Vision/{name}/PoseX")
        public double getPoseX() {
            return getPose().getX();
        }

        // @AutoLogOutput(key = "Vision/{name}/PoseY")
        public double getPoseY() {
            return getPose().getY();
        }

        // @AutoLogOutput(key = "Vision/{name}/TagCount")
        public double getTagCount() {
            return limelight.getTagCountInView();
        }

        // @AutoLogOutput(key = "Vision/{name}/TargetSize")
        public double getTargetSize() {
            return limelight.getTargetSize();
        }
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
