package frc.robot.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.crescendo.Field;
import frc.robot.Robot;
import frc.robot.RobotTelemetry;
import frc.spectrumLib.vision.Limelight;
import frc.spectrumLib.vision.Limelight.PhysicalConfig;
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

        /* Pipeline config */
        public static final int rearDetectorPipeline = 0;
        public static final int speakerDetectorPipeline = 0;

        /* AprilTag Heights (meters) */
        public static final double speakerTagHeight = 1.45;
        public static final int speakerTagID = 4;

        /* Pose Estimation Constants */
        public static final double VISION_REJECT_DISTANCE = 2; // 2.3;
        // Increase these numbers to trust global measurements from vision less.
        public static final double VISION_STD_DEV_X = 0.5;
        public static final double VISION_STD_DEV_Y = 0.5;
        public static final double VISION_STD_DEV_THETA = 99999999;

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

    @AutoLogOutput(key = "Vision/integratingPose")
    public static boolean isPresent = false;

    public Vision() {
        setName("Vision");

        SmartDashboard.putBoolean("VisionPresent", Vision.isPresent);

        /* Configure Limelight Settings Here */
        speakerLL.setLEDMode(false);
        rearLL.setLEDMode(false);
    }

    @Override
    public void periodic() {
        try {
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
            Pose2d botpose = ll.getAlliancePose().toPose2d();
            double timeStamp = getVisionPoseTimestamp(ll);

            // distance from current pose to vision estimated pose
            double poseDifference =
                    Robot.swerve.getPose().getTranslation().getDistance(botpose.getTranslation());

            double targetSize = ll.getTargetSize();
            if (ll.multipleTagsInView()) {
                xyStds = 0.5;
                degStds = 6;
            } else if (targetSize > 0.8 && poseDifference < 0.5) {
                xyStds = 1.0;
                degStds = 12;
            } else if (targetSize > 0.1 && poseDifference < 0.3) {
                xyStds = 2.0;
                degStds = 999999;
            } else {
                RobotTelemetry.print("Vision pose rejected");
                return;
            }

            Robot.swerve.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, degStds));
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
        Translation2d speaker = getAdjustedSpeakerPos();
        Translation2d robotXY = Robot.swerve.getPose().getTranslation();
        double angleBetweenRobotAndSpeaker =
                MathUtil.angleModulus(speaker.minus(robotXY).getAngle().getRadians());

        return angleBetweenRobotAndSpeaker;
    }

    /** Returns the distance from the speaker in meters, adjusted for the robot's movement. */
    @AutoLogOutput
    public double getSpeakerDistance() {
        return Robot.swerve.getPose().getTranslation().getDistance(getAdjustedSpeakerPos());
    }

    /**
     * Gets a field-relative position for the shot to the speaker the robot should take, adjusted
     * for the robot's movement.
     *
     * @return A {@link Translation2d} representing a field relative position in meters.
     */
    public Translation2d getAdjustedSpeakerPos() {
        double NORM_FUDGE = 0.075;
        double tunableNoteVelocity = 5.6;
        double tunableNormFudge = 0.52;
        double tunableStrafeFudge = 0.85;
        double tunableSpeakerYFudge = 0.0;
        double tunableSpeakerXFudge = 0.0;

        Translation2d goalPose = Field.flipXifRed(Field.Speaker.centerSpeakerPose).getTranslation();
        Translation2d robotPos = Robot.swerve.getPose().getTranslation();
        ChassisSpeeds robotVel = Robot.swerve.getVelocity(true);

        double distance = robotPos.getDistance(goalPose);
        double normFactor =
                Math.hypot(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond) < NORM_FUDGE
                        ? 0.0
                        : Math.abs(
                                MathUtil.angleModulus(
                                                robotPos.minus(goalPose).getAngle().getRadians()
                                                        - Math.atan2(
                                                                robotVel.vyMetersPerSecond,
                                                                robotVel.vxMetersPerSecond))
                                        / Math.PI);

        double x =
                goalPose.getX()
                        + (Field.isBlue() ? tunableSpeakerXFudge : -tunableSpeakerXFudge)
                        - (robotVel.vxMetersPerSecond
                                * (distance / tunableNoteVelocity)
                                * (1.0 - (tunableNormFudge * normFactor)));
        double y =
                goalPose.getY()
                        + tunableSpeakerYFudge
                        - (robotVel.vyMetersPerSecond
                                * (distance / tunableNoteVelocity)
                                * tunableStrafeFudge);

        return new Translation2d(x, y);
    }

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

    public double getVisionPoseTimestamp(Limelight ll) {
        return Timer.getFPGATimestamp() - ll.getPoseLatency();
    }

    // we are resetting gyro angle as well?
    public void resetPoseWithVision() {
        // TODO: add more fallback logic here
        Robot.swerve.resetPose(
                Robot.swerve.convertPoseWithGyro(speakerLL.getAlliancePose().toPose2d()));
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
