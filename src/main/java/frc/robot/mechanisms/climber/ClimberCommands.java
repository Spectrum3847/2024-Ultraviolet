package frc.robot.mechanisms.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ClimberCommands {
    private static Climber climber = Robot.climber;

    public static void setupDefaultCommand() {
        climber.setDefaultCommand(
                climber.holdPosition().withName("Climber.default").ignoringDisable(true));
    }

    public static Command fullExtend() {
        return climber.runPosition(climber.config.fullExtend).withName("Climber.fullExtend");
    }

    public static Command home() {
        return climber.runPosition(climber.config.home).withName("Climber.home");
    }

    public static Command raise() {
        return climber.runPercentage(climber.config.raisePercentage);
    }

    public static Command lower() {
        return climber.runPercentage(climber.config.lowerPercentage);
    }

    public static Command topClimb() {
        return climber.runPosition(climber.config.topClimb);
    }

    public static Command midClimb() {
        return climber.runPosition(climber.config.midClimb);
    }

    public static Command botClimb() {
        return climber.runPosition(climber.config.botClimb);
    }

    public static Command coastMode() {
        return climber.coastMode();
    }
}
