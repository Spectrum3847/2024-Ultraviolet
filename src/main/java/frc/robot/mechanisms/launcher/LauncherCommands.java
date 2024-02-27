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

    public static Command runOnDemandVelocity() {
        return new LaunchTest(leftLauncher.config.testVelocity)
                .withName("Launcher.onDemandVelocity");
    }

    public static Command runAmpVelocity() {
        return runLauncherVelocities(leftLauncher.config.ampVelocity, rightLauncher.config.ampVelocity)
                .withName("Launcher.runAmpVelocity");
    }

    public static Command runTest() {
        return runLauncherVelocities(
                        leftLauncher.config.testVelocity, rightLauncher.config.testVelocity)
                .withName("Launcher.runTest");
    }

    public static Command runFull() {
        return runLauncherPercentages(1, 1).withName("Launcher.runFull");
    }

    public static Command launch() {
        return runLauncherVelocities(leftLauncher.config.launch, rightLauncher.config.launch)
                .withName("Launcher.launch");
    }

    public static Command subwoofer() {
        return runLauncherVelocities(leftLauncher.config.subwoofer, rightLauncher.config.subwoofer)
                .withName("Launcher.subwoofer");
    }

    public static Command autonlaunch() {
        return runLauncherVelocities(leftLauncher.config.autonspin, rightLauncher.config.autonspin)
                .withName("Launcher.autonspin");
    }

    public static Command slowLaunchPercent() {
        return runLauncherPercentages(
                        leftLauncher.config.slowLeftLauncherPercentage,
                        rightLauncher.config.slowRightLauncherPercentage)
                .withName("Launcher.slowLaunchPercent");
    }

    public static Command coastMode() {
        return leftLauncher.coastMode().alongWith(rightLauncher.coastMode());
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
