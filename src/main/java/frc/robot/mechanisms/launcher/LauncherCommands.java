package frc.robot.mechanisms.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Robot;
import frc.robot.leds.LEDsConfig;
import frc.robot.leds.LEDsConfig.Section;

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
        return runLauncherPercentages(
                        leftLauncher.config.dumpLuancherPercent,
                        rightLauncher.config.dumpLuancherPercent)
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
        return runLauncherPercentages(
                        leftLauncher.config.ejectLauncherPercent,
                        rightLauncher.config.ejectLauncherPercent)
                .withName("Launcher.eject");
    }

    public static Command coastMode() {
        return leftLauncher.coastMode().alongWith(rightLauncher.coastMode());
    }

    public static Command ensureBrakeMode() {
        return leftLauncher
                .ensureBrakeMode()
                .alongWith(rightLauncher.ensureBrakeMode())
                .withName("EnsureBrakeMode");
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
                .alongWith(
                        rightLauncher.runVelocity(rightVelocity),
                        sendLauncherFeedback(leftVelocity, rightVelocity))
                .withName("Launcher.runLauncherVelocities");
    }

    public static Command runLauncherPercentages(double leftPercentage, double rightPercentage) {
        return leftLauncher
                .runPercentage(leftPercentage)
                .alongWith(rightLauncher.runPercentage(rightPercentage))
                .withName("Launcher.runLauncherPercentages");
    }

    public static Command sendLauncherFeedback(double leftVelocity, double rightVelocity) {
        return new FunctionalCommand(
                () -> {},
                () -> {
                    if (leftLauncher.getMotorVelocityInRPM() >= leftVelocity - 50
                            || rightLauncher.getMotorVelocityInRPM()
                                    >= rightLauncher.getMotorVelocityInRPM() - 50) {
                        Robot.pilot.controller.rumbleController(1, 1);
                        Robot.leds.customStrobe(Section.FULL, LEDsConfig.SPECTRUM_COLOR, 8, 5);
                    }
                },
                (b) -> {
                    Robot.pilot.controller.rumbleController(0, 0);
                    Robot.leds.resetPriority();
                },
                () -> false);
    }

    /* Launcher Specific Commands */

    public static Command stopLeftMotor() {
        return leftLauncher.runStop().withName("LeftLauncher.stopMotor");
    }

    public static Command stopRightMotor() {
        return rightLauncher.runStop().withName("RightLauncher.stopMotor");
    }
}
