package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.spectrumLib.vision.Limelight;

public class Vision extends SubsystemBase {
    public static final class VisionConfig {
        /* Limelight Configuration */
        public static final String DETECT_LL = "limelight-detect";
        public static final String DEFAULT_LL = "limelight-aim";

        /* Pipeline config */
        public static final int noteDetectorPipeline = 1;
        public static final int speakerDetectorPipeline = 1;
    }

    /* Limelights */
    public final Limelight detectLL = new Limelight(VisionConfig.DETECT_LL, VisionConfig.noteDetectorPipeline);
    public final Limelight aimLL = new Limelight(VisionConfig.DEFAULT_LL, VisionConfig.noteDetectorPipeline);

    public Vision() {
        setName("Vision");

        /* Configure Limelights Here */
    }

    @Override
    public void periodic() {}

    /** Change both LL pipelines to the same pipeline */
    public void setLimelightPipelines(int pipeline) {
        detectLL.setLimelightPipeline(pipeline);
        aimLL.setLimelightPipeline(pipeline);
    }

    // /** Resets estimated pose to vision pose */ //TODO: come back; vision
    // public void resetEstimatedPose() {
    //     Robot.pose.resetPoseEstimate(botPose);
    // }

    // /** @return if vision should be trusted more than estimated pose */ //TODO: come back
    // public boolean visionAccurate() {
    //     return isValidPose(botPose) && (isInMap() || multipleTargetsInView());
    // }

    // public boolean isInMap() { //TODO: come back
    //     return ((botPose.getX() > 1.8 && botPose.getX() < 2.5)
    //             && (botPose.getY() > 0.1 && botPose.getY() < 5.49));
    // }
}
