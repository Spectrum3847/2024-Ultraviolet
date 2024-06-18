package frc.robot.mechanisms.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ElevatorCommands {
    private static Elevator elevator = Robot.elevator;

    public static void setupDefaultCommand() {
        elevator.setDefaultCommand(
<<<<<<< HEAD
                elevator.holdPosition().withName("Elevator.default").ignoringDisable(true));
=======
                holdPosition().ignoringDisable(true).withName("Elevator.default"));
    }

    public static Command holdPosition() {
        return elevator.holdPosition().withName("Elevator.holdPosition");
>>>>>>> Madtown-Auto
    }

    public static Command fullExtend() {
        return elevator.runPosition(elevator.config.fullExtend).withName("Elevator.fullExtend");
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

    public static Command ensureBrakeMode() {
        return elevator.ensureBrakeMode();
    }
}
