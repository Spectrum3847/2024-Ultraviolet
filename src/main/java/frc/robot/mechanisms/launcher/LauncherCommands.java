package frc.robot.mechanisms.launcher;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Robot;
import frc.robot.leds.LEDsCommands;
import frc.robot.pilot.PilotCommands;
import java.util.function.DoubleSupplier;

// TODO: the usage will slightly change later
public class LauncherCommands {
    private static LeftLauncher leftLauncher = Robot.leftLauncher;
    private static RightLauncher rightLauncher = Robot.rightLauncher;

    public static final InterpolatingDoubleTreeMap DISTANCE_MAP = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap FEED_DISTANCE_MAP =
            new InterpolatingDoubleTreeMap();

    static {
        // launching
        DISTANCE_MAP.put(1.18, 4500.0);
        DISTANCE_MAP.put(1.76, 4500.0);
        DISTANCE_MAP.put(2.79, 4500.0);
        DISTANCE_MAP.put(2.93, 4500.0);
    }

    public static void setupDefaultCommand() {
        leftLauncher.setDefaultCommand(stopLeftMotor().withName("LeftLauncher.default"));
        rightLauncher.setDefaultCommand(stopRightMotor().withName("RightLauncher.default"));
    }

    /* Launch Commands */

    public static DoubleSupplier getRPMfromDistance(DoubleSupplier distance) {
        return () -> DISTANCE_MAP.get(distance.getAsDouble());
    }

    public static Command runPercentOutput(double output) {
        return leftLauncher.runPercentage(output).alongWith(rightLauncher.runPercentage(output));
    }

    public static Command distanceVelocity(DoubleSupplier distance) {
        return velocityTCFOCrpm(getRPMfromDistance(distance));
    }

    public static Command velocityTCFOCrpm(DoubleSupplier velocityRPM) {
        return leftLauncher
                .runVelocityTCFOCrpm(velocityRPM)
                .alongWith(
                        rightLauncher.runVelocityTCFOCrpm(velocityRPM),
                        sendLauncherFeedback(velocityRPM.getAsDouble(), velocityRPM.getAsDouble()));
    }

    public static Command runOnDemandVelocity() {
        return new LaunchTest(leftLauncher.config.testVelocity)
                .withName("Launcher.onDemandVelocity");
    }

    public static Command runTest() {
        return runLauncherVelocities(
                        leftLauncher.config.testVelocity, rightLauncher.config.testVelocity)
                .withName("Launcher.runTest");
    }

    public static Command runFull() {
        return runLauncherVelocities(leftLauncher.config.maxSpeed, rightLauncher.config.maxSpeed)
                .withName("Launcher.runFull");
    }

    public static Command runAmpVelocity() {
        return runPercentOutput(0.45)
                // runLauncherVelocities(leftLauncher.config.ampVelocity,
                // rightLauncher.config.ampVelocity)
                .withName("Launcher.runAmpVelocity");
    }

    public static Command eject() {
        return runLauncherPercentages(-.4, -.4).withName("Launcher.ejectSequence");
    }

    public static Command launch() {
        return runLauncherVelocities(leftLauncher.config.launch, rightLauncher.config.launch)
                .withName("Launcher.launch");
    }

    public static Command subwoofer() {
        return runPercentOutput(1).withName("Launcher.subwoofer");
    }

    public static Command launchReadyPreload() {
        return runLauncherVelocities(
                leftLauncher.config.launchReadyPreload, rightLauncher.config.launchReadyPreload);
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
                .runVelocityTorqueCurrentFOC(leftVelocity)
                .alongWith(rightLauncher.runVelocityTorqueCurrentFOC(rightVelocity))
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
                            || rightLauncher.getMotorVelocityInRPM() >= rightVelocity - 50) {
                        PilotCommands.rumble(2.0, 1.0).alongWith(LEDsCommands.launchReady());
                    }
                },
                (b) -> {
                    PilotCommands.rumble(0, 0);
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

    public static Command LeftLauncherTest() {
        return leftLauncher.runPercentage(leftLauncher.config.testForwardPercent);
    }

    public static Command runVelocityTestin() {
        return runLauncherVelocities(
                        leftLauncher.config.testVelocity, rightLauncher.config.testVelocity)
                .withName("Launcher.runTestVelocity");
    }
}
