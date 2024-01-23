package frc.robot.mechanisms.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ClimberCommands {
    private static Climber climber = Robot.climber;

    public static void setupDefaultCommand() {
        climber.setDefaultCommand(climber.holdPosition().withName("Climber.default"));
    }

    public static Command fullExtend() {
        return climber.runPosition(climber.config.fullExtend).withName("Climber.fullExtend");
    }

    public static Command home() {
        return climber.runPosition(climber.config.home).withName("Climber.home");
    }
}
