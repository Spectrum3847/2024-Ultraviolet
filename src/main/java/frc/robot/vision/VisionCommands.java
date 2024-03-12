package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.vision.Vision.VisionConfig.AlignToAmp;
import frc.robot.vision.Vision.VisionConfig.AlignToSpeaker;
import frc.robot.vision.Vision.VisionConfig.AlignToStage;
import frc.robot.vision.commands.AlignToVisionTarget;
import frc.robot.vision.commands.AlignWithPose;

public class VisionCommands {
    private static Vision vision = Robot.vision;

    public static Command alignToSpeaker() {
        return new AlignToVisionTarget(
                AlignToSpeaker.getConfig(), () -> Robot.pilot.getDriveFwdPositive(), 0);
    }

    public static Command alignToAmp() {
        return new AlignWithPose(
                AlignToAmp.getConfig(), () -> Robot.pilot.getDriveLeftPositive(), 0);
    }

    // public static Command alignToAmp() {
    //     return new AlignToVisionTarget(
    //             AlignToAmp.getConfig(), () -> Robot.pilot.getDriveFwdPositive(), 0);
    // }

    public static Command alignToStage() {
        return new AlignToVisionTarget(
                AlignToStage.getConfig(), () -> Robot.pilot.getDriveFwdPositive(), 0);
    }

    public static Command blinkLimelights() {
        return vision.blinkLimelights().withName("VisionCommands.blinkLimelights");
    }

    public static Command resetPoseWithVision() {
        return vision.run(vision::resetPoseWithVision)
                .withName("VisionCommands.resetPoseWithVision");
    }
}
