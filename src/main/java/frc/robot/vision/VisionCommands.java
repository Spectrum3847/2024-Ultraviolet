package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.vision.Vision.VisionConfig.AlignToNote;
import frc.robot.vision.Vision.VisionConfig.DriveToNote;
import frc.robot.vision.commands.AlignToVisionTarget;
import frc.robot.vision.commands.DriveToVisionTarget;

public class VisionCommands {
    private static Vision vision = Robot.vision;

    public static Command alignToNote() {
        return new AlignToVisionTarget(AlignToNote.getConfig(), () -> 0, 0)
                .withName("VisionCommands.alignToNote");
    }

    public static Command driveToNote() {
        return new DriveToVisionTarget(DriveToNote.getConfig(), 0)
                .withName("VisionCommands.driveToNote");
    }

    public static Command blinkLimelights() {
        return vision.blinkLimelights().withName("VisionCommands.blinkLimelights");
    }

    public static Command resetPoseWithVision() {
        return vision.run(vision::resetPoseWithVision)
                .withName("VisionCommands.resetPoseWithVision");
    }
}
