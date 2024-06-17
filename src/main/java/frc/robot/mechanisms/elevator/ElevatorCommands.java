package frc.robot.mechanisms.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ElevatorCommands {
    private static Elevator elevator = Robot.elevator;

    public static void setupDefaultCommand() {
        elevator.setDefaultCommand(
                elevator.holdPosition().withName("Elevator.default").ignoringDisable(true));
    }

    public static Command fullExtend() {
        return elevator.runPosition(elevator.config.fullExtend).withName("Elevator.fullExtend");
    }

    public static Command percentage() {
        return elevator.runPercentage(0.3);
    }

    public static Command amp() {
        return elevator.runPosition(elevator.config.amp).withName("Elevator.amp");
    }

    public static Command trap() {
        return elevator.runPosition(elevator.config.trap).withName("Elevator.trap");
    }

    public static Command home() {
        return elevator.runPosition(elevator.config.home).withName("Elevator.home");
    }

    public static Command coastMode() {
        return elevator.coastMode();
    }
}
