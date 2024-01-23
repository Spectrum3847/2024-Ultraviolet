package frc.robot.operator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;
import frc.spectrumLib.util.exceptions.KillRobotException;

/** This class should have any command calls that directly call the Operator */
public class OperatorCommands {
    private static Operator operator = Robot.operator;

    /** Set default command to turn off the rumble */
    public static void setupDefaultCommand() {
        operator.setDefaultCommand(rumble(0, 99999).repeatedly().withName("Operator.default"));
    }

    /** Command that can be used to rumble the Operator controller */
    public static Command rumble(double intensity, double durationSeconds) {
        return operator.rumbleCommand(intensity, durationSeconds);
    }

    public static Command cancelCommands() {
        return new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())
                .withName("OperatorCancelAll");
    }

    public static Command killTheRobot() {
        return new InstantCommand(() -> operatorError()).ignoringDisable(true);
    }

    public static void operatorError() {
        throw new KillRobotException("The robot was killed by operator");
    }

    //TODO: temporary manual commands for mechanisms; usage will change

    public static Command manualAmpTrap() {
        return new RunCommand(() -> Robot.ampTrap.runPercentage(operator.controller.getLeftY()), Robot.ampTrap).withName("AmpTrap.manualOutput");
    }

    public static Command manualClimber() {
        return new RunCommand(() -> Robot.climber.runPercentage(operator.controller.getLeftY()), Robot.climber).withName("Climber.manualOutput");
    }

    public static Command manualElevator() {
        return new RunCommand(() -> Robot.elevator.runPercentage(operator.controller.getLeftY()), Robot.elevator).withName("Elevator.manualOutput");
    }

    public static Command manualFeeder() {
        return new RunCommand(() -> Robot.feeder.runPercentage(operator.controller.getLeftY()), Robot.feeder).withName("Feeder.manualOutput");
    }

    public static Command manualIntake() {
        return new RunCommand(() -> Robot.intake.runPercentage(operator.controller.getLeftY()), Robot.intake).withName("Intake.manualOutput");
    }

    public static Command manualPivot() {
        return new RunCommand(() -> Robot.pivot.runManualOutput(operator.controller.getLeftY()), Robot.pivot).withName("Pivot.manualOutput");
    }

    public static Command manualLauncher() {
        return new RunCommand(() -> {
            Robot.leftLauncher.runPercentage(operator.controller.getLeftY());
            Robot.rightLauncher.runPercentage(operator.controller.getLeftY());
        }, Robot.leftLauncher, Robot.rightLauncher).withName("Launcher.manualOutput");
    }
}
