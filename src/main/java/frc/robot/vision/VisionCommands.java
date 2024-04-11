package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class VisionCommands {
    private static Vision vision = Robot.vision;

    public static Command blinkLimelights() {
        return vision.blinkLimelights().withName("VisionCommands.blinkLimelights");
    }

    public static Command solidLimelight() {
        return vision.solidLimelight().withName("VisionCommands.solidLimelight");
    }
    /** Set robot pose to vision pose only if LL has good tag reading */
    public static Command resetPoseToVision() {
        return vision.runOnce(vision::resetPoseToVision)
                .withName("VisionCommands.resetPoseToVision");
    }

    /** Set robot pose to vision pose regardless of validity. Does not reset rotation. */
    public static Command forcePoseToVision() {
        return vision.run(vision::forcePoseToVision).withName("VisionCommands.forcePoseToVision");
    }
}
