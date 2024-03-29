package frc.spectrumLib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.vision.Vision.VisionConfig;
import frc.spectrumLib.vision.LimelightHelpers.LimelightResults;
import frc.spectrumLib.vision.LimelightHelpers.RawFiducial;
import java.text.DecimalFormat;
import java.util.Optional;

public class Limelight {

    /* Limelight Configuration */

    /** Must match to the name given in LL dashboard */
    public final String CAMERA_NAME;

    public String logStatus = "";
    public String tagStatus = "";
    public boolean trustStrong = false;
    /** Physical Config */
    private PhysicalConfig physicalConfig;

    /* Debug */
    private final DecimalFormat df = new DecimalFormat();

    public Limelight(String cameraName) {
        this.CAMERA_NAME = cameraName;
        physicalConfig = new PhysicalConfig();
        logStatus = "Not started";
        tagStatus = "Not started";
        trustStrong = false;
    }

    public Limelight(String cameraName, int pipeline) {
        this(cameraName);
        setLimelightPipeline(pipeline);
    }

    public Limelight(String cameraName, int pipeline, PhysicalConfig physicalConfig) {
        this(cameraName, pipeline);
        this.physicalConfig = physicalConfig;
        LimelightHelpers.setCameraPose_RobotSpace(
                this.CAMERA_NAME,
                physicalConfig.forward,
                physicalConfig.right,
                physicalConfig.up,
                physicalConfig.roll,
                physicalConfig.pitch,
                physicalConfig.yaw);
    }

    /*
     *
     * Frequently Used Methods
     *
     *
     */

    /* ::: Basic Information Retrieval ::: */

    /**
     * @return Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2:
     *     -29.8 to 29.8 degrees)
     */
    public double getHorizontalOffset() {
        return LimelightHelpers.getTX(CAMERA_NAME);
    }

    /**
     * @return Vertical Offset From Crosshair To Target in degrees (LL1: -20.5 degrees to 20.5
     *     degrees / LL2: -24.85 to 24.85 degrees)
     */
    public double getVerticalOffset() {
        return LimelightHelpers.getTY(CAMERA_NAME);
    }

    /** @return Whether the LL has any valid targets (apriltags or other vision targets) */
    public boolean targetInView() {
        return LimelightHelpers.getTV(CAMERA_NAME);
    }

    /** @return whether the LL sees multiple tags or not */
    public boolean multipleTagsInView() {
        return getTagCountInView() > 1;
    }

    public double getTagCountInView() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(CAMERA_NAME).tagCount;

        // if (retrieveJSON() == null) return 0;

        // return retrieveJSON().targetingResults.targets_Fiducials.length;
    }

    /**
     * @return the tag ID of the apriltag most centered in the LL's view (or based on different
     *     criteria set in LL dasbhoard)
     */
    public double getClosestTagID() {
        return LimelightHelpers.getFiducialID(CAMERA_NAME);
    }

    public double getTargetSize() {
        return LimelightHelpers.getTA(CAMERA_NAME);
    }

    /* ::: Pose Retrieval ::: */

    /** @return the corresponding LL Pose3d for the alliance in DriverStation.java */
    public Pose3d getRawPose3d() {
        return LimelightHelpers.getBotPose3d_wpiBlue(
                CAMERA_NAME); // 2024: all alliances use blue as 0,0
    }

    public boolean hasAccuratePose() {
        return multipleTagsInView() && getTargetSize() > 0.1;
    }

    public Optional<Pose2d> getFilteredPose(Pose2d swervePose, double rejectDistance) {
        Pose2d visionPose = getRawPose3d().toPose2d();

        // if no camera or no target in view, return empty
        if (!isCameraConnected() || !targetInView()) {
            logStatus = "No Apriltag in view";
            return Optional.empty();
        }
        // if vision pose is too far off current and we are not very close to a tag, ignore it
        if (swervePose.getTranslation().getDistance(visionPose.getTranslation()) < rejectDistance
                || (swervePose.getX() <= 0 || Robot.swerve.getPose().getY() <= 0)
                || (getDistanceToTagFromCamera() <= 1)
                || (getTagCountInView() >= 2 && getDistanceToTagFromCamera() <= 3)) {
            return Optional.of(new Pose2d(visionPose.getTranslation(), swervePose.getRotation()));
        }

        // dumb
        if (swervePose.getTranslation().getDistance(visionPose.getTranslation()) > rejectDistance) {
            logStatus = "Rejected: Too far off odometry";
        }

        logStatus = "Unknown error";
        return Optional.empty();
    }

    /** @return the distance of the 2d vector from the camera to closest apriltag */
    public double getDistanceToTagFromCamera() {
        double x = LimelightHelpers.getCameraPose3d_TargetSpace(CAMERA_NAME).getX();
        double y = LimelightHelpers.getCameraPose3d_TargetSpace(CAMERA_NAME).getZ();
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public RawFiducial[] getRawFiducial() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(CAMERA_NAME).rawFiducials;
    }

    /**
     * Returns the timestamp of the pose estimation from the Limelight camera.
     *
     * @return The timestamp of the pose estimation in seconds.
     */
    public double getVisionPoseTimestamp() {
        return Timer.getFPGATimestamp() - getPoseLatency();
    }

    /**
     * Returns the latency of the pose estimation from the Limelight camera.
     *
     * @return The latency of the pose estimation in seconds.
     */
    public double getPoseLatency() {
        return Units.millisecondsToSeconds(LimelightHelpers.getBotPose_wpiBlue(CAMERA_NAME)[6]);
    }

    /*
     *
     * Custom Helpers
     *
     *
     */

    /**
     * get distance in meters to a target
     *
     * @param targetHeight meters
     * @return
     */
    public double getDistanceToTarget(double targetHeight) {
        return (targetHeight - physicalConfig.up)
                / Math.tan(Units.degreesToRadians(physicalConfig.roll + getVerticalOffset()));
    }

    /*
     *
     * Utility Wrappers
     *
     *
     */

    /** @return The latest LL results as a LimelightResults object. */
    private LimelightResults retrieveJSON() {
        return LimelightHelpers.getLatestResults(CAMERA_NAME);
    }

    /** @param pipelineIndex use pipeline indexes in {@link VisionConfig} //TODO: come back */
    public void setLimelightPipeline(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex(CAMERA_NAME, pipelineIndex);
    }

    /**
     * Sets the LED mode of the LL.
     *
     * @param enabled true to enable the LED mode, false to disable it
     */
    public void setLEDMode(boolean enabled) {
        if (enabled) {
            LimelightHelpers.setLEDMode_ForceOn(CAMERA_NAME);
        } else {
            LimelightHelpers.setLEDMode_ForceOff(CAMERA_NAME);
        }
    }

    /**
     * Set LL LED's to blink
     *
     * @return
     */
    public void blinkLEDs() {
        LimelightHelpers.setLEDMode_ForceBlink(CAMERA_NAME);
    }

    /** Checks if the camera is connected by looking for an empty botpose array from camera. */
    public boolean isCameraConnected() {
        try {
            var rawPoseArray =
                    LimelightHelpers.getLimelightNTTableEntry(CAMERA_NAME, "botpose_wpiblue")
                            .getDoubleArray(new double[0]);
            if (rawPoseArray.length < 6) {
                return false;
            }
            return true;
        } catch (Exception e) {
            System.err.println("Avoided crashing statement in Limelight.java: isCameraConnected()");
            return false;
        }
    }

    /** Prints the vision, estimated, and odometry pose to SmartDashboard */
    public void printDebug() {
        Pose3d botPose3d = getRawPose3d();
        SmartDashboard.putString("LimelightX", df.format(botPose3d.getTranslation().getX()));
        SmartDashboard.putString("LimelightY", df.format(botPose3d.getTranslation().getY()));
        SmartDashboard.putString("LimelightZ", df.format(botPose3d.getTranslation().getZ()));
        SmartDashboard.putString(
                "LimelightRoll", df.format(Units.radiansToDegrees(botPose3d.getRotation().getX())));
        SmartDashboard.putString(
                "LimelightPitch",
                df.format(Units.radiansToDegrees(botPose3d.getRotation().getY())));
        SmartDashboard.putString(
                "LimelightYaw", df.format(Units.radiansToDegrees(botPose3d.getRotation().getZ())));
    }

    // TODO: isEstimateReady, toPose2d

    /**
     * Specify the location of your Limelight relative to the center of your robot. (meters,
     * degrees)
     */
    public static class PhysicalConfig {
        public double forward, right, up; // meters
        public double roll, pitch, yaw; // degrees

        /**
         * Specify the location of your Limelight relative to the center of your robot. (meters,
         * degrees)
         */
        public PhysicalConfig() {}

        /**
         * @param forward (meters) forward from center of robot
         * @param right (meters) right from center of robot
         * @param up (meters) up from center of robot
         * @return
         */
        public PhysicalConfig withTranslation(double forward, double right, double up) {
            this.forward = forward;
            this.right = right;
            this.up = up;
            return this;
        }

        /**
         * @param roll (degrees) roll of limelight || positive is rotated right
         * @param pitch (degrees) pitch of limelight || positive is camera tilted up
         * @param yaw (yaw) yaw of limelight || positive is rotated left
         * @return
         */
        public PhysicalConfig withRotation(double roll, double pitch, double yaw) {
            this.roll = roll;
            this.pitch = pitch;
            this.yaw = yaw;
            return this;
        }
    }
}
