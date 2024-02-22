package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.spectrumLib.vision.Limelight;
import frc.spectrumLib.vision.Limelight.PhysicalConfig;

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

    public Vision() {
        setName("Vision");

        /* Configure Limelight Settings Here */
    }

    @Override
    public void periodic() {}

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
