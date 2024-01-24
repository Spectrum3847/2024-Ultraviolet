package frc.robot.mechanisms.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

// TODO: the usage will slightly change later
public class LauncherCommands {
    private static LeftLauncher leftLauncher = Robot.leftLauncher;
    private static RightLauncher rightLauncher = Robot.rightLauncher;

    public static void setupDefaultCommand() {
        leftLauncher.setDefaultCommand(stopLeftMotor().withName("LeftLauncher.default"));
        rightLauncher.setDefaultCommand(stopRightMotor().withName("RightLauncher.default"));
    }

    /* Launch Commands */

    public static Command runFull() {
        return runLauncherVelocities(leftLauncher.config.maxSpeed, rightLauncher.config.maxSpeed)
                .withName("Launcher.runFull");
    }

    public static Command launch() {
        return runLauncherVelocities(leftLauncher.config.launch, rightLauncher.config.launch)
                .withName("Launcher.launch");
    }

    public static Command slowLaunchPercent() {
        return runLauncherPercentages(
                        leftLauncher.config.slowLeftLauncherPercentage,
                        rightLauncher.config.slowRightLauncherPercentage)
                .withName("Launcher.slowLaunchPercent");
    }

    /* Helpers */

    public static Command stopMotors() {
        return stopLeftMotor().alongWith(stopRightMotor()).withName("Launcher.stopMotor");
    }

    public static Command runLauncherVelocities(double leftVelocity, double rightVelocity) {
        return leftLauncher
                .runVelocity(leftVelocity)
                .alongWith(rightLauncher.runVelocity(rightVelocity))
                .withName("Launcher.runLauncherVelocities");
    }

    public static Command runLauncherPercentages(double leftPercentage, double rightPercentage) {
        return leftLauncher
                .runPercentage(leftPercentage)
                .alongWith(rightLauncher.runPercentage(rightPercentage))
                .withName("Launcher.runLauncherPercentages");
    }

    /* Launcher Specific Commands */

    public static Command stopLeftMotor() {
        return leftLauncher.runStop().withName("LeftLauncher.stopMotor");
    }

    public static Command stopRightMotor() {
        return rightLauncher.runStop().withName("RightLauncher.stopMotor");
    }

    public static Command LeftLauncherTest() {
        return leftLauncher.runPercentage(leftLauncher.config.testForwardPercent);
    }

    public static Command runVelocityTestin() {
        return runLauncherVelocities(
                        leftLauncher.config.testVelocity, rightLauncher.config.testVelocity)
                .withName("Launcher.runTestVelocity");
    }
}
