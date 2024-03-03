package frc.robot.mechanisms.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class LauncherCommands {
    private static LeftLauncher leftLauncher = Robot.leftLauncher;
    private static RightLauncher rightLauncher = Robot.rightLauncher;

    public static void setupDefaultCommand() {
        leftLauncher.setDefaultCommand(stopLeftMotor().withName("LeftLauncher.default"));
        rightLauncher.setDefaultCommand(stopRightMotor().withName("RightLauncher.default"));
    }

    /* Launch Commands */

    public static Command runOnDemandVelocity() {
        return new OnDemandLaunch(leftLauncher.config.testVelocity)
                .withName("Launcher.onDemandVelocity");
    }

    public static Command launch() {
        return runLauncherVelocities(leftLauncher.config.launch, rightLauncher.config.launch)
                .withName("Launcher.launch");
    }

    public static Command dump() {
        return runLauncherVelocities(leftLauncher.config.dump, rightLauncher.config.dump)
                .withName("Launcher.dump");
    }

    public static Command subwoofer() {
        return runLauncherVelocities(leftLauncher.config.subwoofer, rightLauncher.config.subwoofer)
                .withName("Launcher.subwoofer");
    }

    public static Command deepShot() {
        return runLauncherVelocities(leftLauncher.config.deepShot, rightLauncher.config.deepShot)
                .withName("Launcher.deepShot");
    }

    public static Command eject() {
        return runLauncherVelocities(leftLauncher.config.eject, rightLauncher.config.eject)
                .withName("Launcher.eject");
    }

    public static Command coastMode() {
        return leftLauncher.coastMode().alongWith(rightLauncher.coastMode());
    }

    /* Helpers */

    public static Command stopMotors() {
        return stopLeftMotor().alongWith(stopRightMotor()).withName("Launcher.stopMotor");
    }

    public static Command runLauncherVelocities(double velocity) {
        return runLauncherVelocities(velocity, velocity);
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
}
