package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.spectrumLib.vision.Limelight;

public class Vision extends SubsystemBase {
    public static final class VisionConfig {
        /* Limelight Configuration */
        public static final String NOTE_LL = "limelight-detect"; //TODO: change this name in LL dashboard to reflect the name here
        public static final String SPEAKER_LL = "limelight-aim"; //TODO: change this name in LL dashboard to reflect the name here

        /* Pipeline config */
        public static final int noteDetectorPipeline = 1;
        public static final int speakerDetectorPipeline = 1;
    }

    /* Limelights */
    public final Limelight noteLL = new Limelight(VisionConfig.NOTE_LL, VisionConfig.noteDetectorPipeline);
    public final Limelight speakerLL = new Limelight(VisionConfig.SPEAKER_LL, VisionConfig.speakerDetectorPipeline);

    public Vision() {
        setName("Vision");

        /* Configure Limelight Settings Here */
    }

    @Override
    public void periodic() {}

    /**
     * Calculates the required rotation for the robot to align with a note, based on the current orientation of the robot and the note's positional offset from the camera's center. The result is the angle in degrees that the robot needs to turn to face the note directly.
     * @return The angle in degrees to rotate the robot towards the note.
     */    
    public double getOffsetToNote() {
        return Robot.swerve.getRotation().getDegrees() - noteLL.getHorizontalOffset();
    }

    /**
     * Calculates the required rotation for the robot to align with the speaker, based on the current orientation of the robot and the speaker's positional offset from the camera's center. The result is the angle in degrees that the robot needs to turn to face the speaker directly.
     * @return The angle in degrees to rotate the robot towards the speaker.
     */    
    public double getOffsetToSpeaker() {
        return Robot.swerve.getRotation().getDegrees() - speakerLL.getHorizontalOffset();
    }

    /** Change both LL pipelines to the same pipeline */
    public void setLimelightPipelines(int pipeline) {
        noteLL.setLimelightPipeline(pipeline);
        speakerLL.setLimelightPipeline(pipeline);
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
