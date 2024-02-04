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


        /* Vision Command Configs */
        public static final class AlignToNote extends CommandConfig {
            public AlignToNote() {
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

        public static final class DriveToNote extends CommandConfig {
            public DriveToNote() {
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

        public CommandConfig() {

        }
        
    } 
}
