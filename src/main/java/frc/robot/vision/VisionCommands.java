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

    /** Set robot pose to vision pose looking at 5 of the last available poses in auto */
    public static Command autonResetPoseToVision() {
        return vision.runOnce(vision::autonResetPoseToVision)
                .withName("VisionCommands.autonResetPoseToVision");
    }

    public static Command forcePoseToVision() {
        return Robot.vision
                .runOnce(Robot.vision::forcePoseToVision)
                .ignoringDisable(true)
                .withName("VisionCommands.forcePoseToVision");
    }
}
