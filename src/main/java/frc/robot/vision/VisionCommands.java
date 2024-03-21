package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class VisionCommands {

    public static Command forcePoseToVision() {
        return Robot.vision
                .runOnce(Robot.vision::forcePoseToVision)
                .ignoringDisable(true)
                .withName("VisionCommands.forcePoseToVision");
    }
}
