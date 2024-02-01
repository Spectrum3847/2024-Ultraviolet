package frc.robot.mechanisms.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;

public class ClimberCommands {
    private static Climber climber = Robot.climber;

    public static void setupDefaultCommand() {
        climber.setDefaultCommand(climber.runPercentage(0.0).withName("Climber.default"));
    }

    public static Command fullExtend() {
        return climber.runPosition(climber.config.fullExtend).withName("Climber.fullExtend");
    }

    public static Command home() {
        return climber.runPosition(climber.config.home).withName("Climber.home");
    }

    public static Command raise() {
        return climber.runPercentage(0.2);
    }

    public static Command lower() {
        return climber.runPercentage(-0.2);
    }

    public static Command manualCommand(DoubleSupplier stick) {
        return climber.runPercentage(stick);
    }
}
