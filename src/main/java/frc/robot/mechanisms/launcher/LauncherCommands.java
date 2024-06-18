package frc.robot.mechanisms.launcher;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Robot;
import frc.robot.RobotTelemetry;
import frc.robot.leds.LEDsCommands;
import frc.robot.pilot.PilotCommands;
import java.util.function.DoubleSupplier;

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
        leftLauncher.setDefaultCommand(
                stopLeftMotor().withName("LeftLauncher.default").ignoringDisable(true));
        rightLauncher.setDefaultCommand(
                stopRightMotor().withName("RightLauncher.default").ignoringDisable(true));
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
        return new OnDemandLaunch(leftLauncher.config.testVelocity)
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
        return runLauncherVelocities(950, 1250)
                // runLauncherVelocities(leftLauncher.config.ampVelocity,
                // rightLauncher.config.ampVelocity)
                .withName("Launcher.runAmpVelocity");
    }

    public static Command launch() {
        return runLauncherVelocities(leftLauncher.config.launch, rightLauncher.config.launch)
                .withName("Launcher.launch");
    }

    public static Command intoAmp() {
        return runLauncherVelocities(leftLauncher.config.intoAmp, rightLauncher.config.intoAmp)
                .withName("Launcher.intoAmp");
    }

    public static Command dump() {
        return runLauncherPercentages(
                        leftLauncher.config.dumpLauncherPercent,
                        rightLauncher.config.dumpLauncherPercent)
                .withName("Launcher.dump");
    }

    public static Command autoDump() {
        return runLauncherPercentages(
                        leftLauncher.config.autoDumpLauncherPercent,
                        rightLauncher.config.autoDumpLauncherPercent)
                .withName("Launcher.autoDump");
    }

    public static Command autoDump2() {
        return runLauncherPercentages(
                        leftLauncher.config.auto2DumpLauncherPercent,
                        rightLauncher.config.auto2DumpLauncherPercent)
                .withName("Launcher.autoDump2");
    }

    public static Command autoDump3() {
        return runLauncherPercentages(
                        leftLauncher.config.auto3DumpLauncherPercent,
                        rightLauncher.config.auto3DumpLauncherPercent)
                .withName("Launcher.autoDump2");
    }

    public static Command autoDump4() {
        return runLauncherPercentages(
                        leftLauncher.config.auto4DumpLauncherPercent,
                        rightLauncher.config.auto4DumpLauncherPercent)
                .withName("Launcher.autoDump2");
    }

    public static Command subwoofer() {
        return runPercentOutput(1).withName("Launcher.subwoofer");
    }

    public static Command launchReadyPreload() {
        return runLauncherVelocities(
                leftLauncher.config.launchReadyPreload, rightLauncher.config.launchReadyPreload);
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
                .withName("Launcher.ensureBrakeMode");
    }

    /* Helpers */

    public static double getMapRPM(InterpolatingDoubleTreeMap map, double distance) {
        double RPM = map.get(distance);
        RobotTelemetry.print(
                "VisionLaunching: interpolating "
                        + RobotTelemetry.truncatedDouble(RPM)
                        + " RPM from "
                        + RobotTelemetry.truncatedDouble(distance)
                        + " meters");
        return RPM;
    }

    public static Command stopMotors() {
        return stopLeftMotor().alongWith(stopRightMotor()).withName("Launcher.stopMotor");
    }

    public static Command runLauncherVelocities(double velocity) {
        return runLauncherVelocities(velocity, velocity);
    }

    public static Command runLauncherVelocities(double leftVelocity, double rightVelocity) {
        return leftLauncher
                .runVelocityTCFOC(leftVelocity)
                .alongWith(rightLauncher.runVelocityTCFOC(rightVelocity))
                .withName("Launcher.runLauncherVelocities");
    }

    public static Command runTorqueLauncherVelocities(double leftVelocity, double rightVelocity) {
        return leftLauncher
                .runVelocityTCFOC(leftVelocity)
                .alongWith(
                        rightLauncher.runVelocity(rightVelocity),
                        sendLauncherFeedback(leftVelocity, rightVelocity))
                .withName("Launcher.runTorqueLaunherVelocities");
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
}
