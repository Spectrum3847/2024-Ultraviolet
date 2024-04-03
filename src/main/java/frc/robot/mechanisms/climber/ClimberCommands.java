package frc.robot.mechanisms.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ClimberCommands {
    private static Climber climber = Robot.climber;

    public static void setupDefaultCommand() {
        climber.setDefaultCommand(
                climber.holdPosition().ignoringDisable(true).withName("Climber.default"));
    }

    public static Command fullExtend() {
        return climber.runPosition(climber.config.fullExtend).withName("Climber.fullExtend");
    }

    public static Command home() {
        return climber.runPosition(climber.config.home).withName("Climber.home");
    }

    public static Command topClimb() {
        return climber.runPosition(climber.config.topClimb).withName("Climber.topClimb");
    }

    public static Command midClimb() {
        return climber.runPosition(climber.config.midClimb).withName("Climber.midClimb");
    }

    public static Command botClimb() {
        return climber.runPosition(climber.config.botClimb).withName("Climber.botClimb");
    }

    public static Command safeClimb() {
        return climber.runPosition(climber.config.safeClimb).withName("Climber.safeClimb");
    }

    public static Command coastMode() {
        return climber.coastMode();
    }

    public static Command ensureBrakeMode() {
        return climber.ensureBrakeMode();
    }
}
