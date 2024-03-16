package frc.robot.mechanisms.launcher;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;

public class LauncherCommands {
    private static LeftLauncher leftLauncher = Robot.leftLauncher;
    private static RightLauncher rightLauncher = Robot.rightLauncher;

    public static boolean isAtSpeed = false;

    public static final InterpolatingDoubleTreeMap DISTANCE_MAP = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap FEED_DISTANCE_MAP =
            new InterpolatingDoubleTreeMap();

    static {
        // launching
        DISTANCE_MAP.put(1.505, 3500.0);
        DISTANCE_MAP.put(2.629, 4500.0);
        DISTANCE_MAP.put(3.969, 4500.0);
        DISTANCE_MAP.put(4.269, 5200.0);
        DISTANCE_MAP.put(4.899, 5200.0);
        DISTANCE_MAP.put(5.189, 5200.0);
        DISTANCE_MAP.put(5.829, 5200.0);
        DISTANCE_MAP.put(6.229, 5200.0);

        // feed launching
        FEED_DISTANCE_MAP.put(6.0, 2600.0);
        FEED_DISTANCE_MAP.put(6.08, 2600.0);
        FEED_DISTANCE_MAP.put(6.47, 2700.0);
        FEED_DISTANCE_MAP.put(6.96, 2700.0);
        FEED_DISTANCE_MAP.put(7.54, 3000.0);
        FEED_DISTANCE_MAP.put(7.74, 3000.0);
        FEED_DISTANCE_MAP.put(9.05, 3800.0);
    }

    public static void setupDefaultCommand() {
        leftLauncher.setDefaultCommand(stopLeftMotor().withName("LeftLauncher.default"));
        rightLauncher.setDefaultCommand(stopRightMotor().withName("RightLauncher.default"));
    }

    /* Launch Commands */

    public static DoubleSupplier getRPMfromDistance(DoubleSupplier distance) {
        return () -> DISTANCE_MAP.get(distance.getAsDouble());
    }

    public static DoubleSupplier getRPMFromFeedDistance(DoubleSupplier distance) {
        return () -> FEED_DISTANCE_MAP.get(distance.getAsDouble());
    }

    public static Command distanceVelocity(DoubleSupplier distance) {
        return velocityTCFOCrpm(getRPMfromDistance(distance));
    }

    public static Command feedDistanceVelocity(DoubleSupplier distance) {
        return velocityTCFOCrpm(getRPMFromFeedDistance(distance));
    }

    public static Command velocityTCFOCrpm(DoubleSupplier velocityRPM) {
        return leftLauncher
                .runVelocityTCFOCrpm(velocityRPM)
                .alongWith(
                        rightLauncher.runVelocityTCFOCrpm(velocityRPM),
                        sendLauncherFeedback(velocityRPM));
    }

    public static Command runOnDemandVelocity() {
        return new OnDemandLaunch(leftLauncher.config.testVelocity)
                .withName("Launcher.onDemandVelocity");
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
                        leftLauncher.config.dumpLuancherPercent,
                        rightLauncher.config.dumpLuancherPercent)
                .withName("Launcher.dump");
    }

    public static Command subwoofer() {
        return runTorqueLauncherVelocities(
                        leftLauncher.config.subwoofer, rightLauncher.config.subwoofer)
                .withName("Launcher.subwoofer");
    }

    public static Command deepShot() {
        return runTorqueLauncherVelocities(
                        leftLauncher.config.deepShot, rightLauncher.config.deepShot)
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
                .withName("Launcher.ensureBrakeMode");
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

    public static Command sendLauncherFeedback(double velocity) {
        return sendLauncherFeedback(() -> velocity, () -> velocity);
    }

    public static Command sendLauncherFeedback(double leftVelocity, double rightVelocity) {
        return sendLauncherFeedback(() -> leftVelocity, () -> rightVelocity);
    }

    public static Command sendLauncherFeedback(DoubleSupplier velocity) {
        return sendLauncherFeedback(velocity, velocity);
    }

    public static Command sendLauncherFeedback(
            DoubleSupplier leftVelocity, DoubleSupplier rightVelocity) {
        return new FunctionalCommand(
                () -> {},
                () -> {
                    if (leftLauncher.getMotorVelocityInRPM() >= leftVelocity.getAsDouble() - 50
                            || rightLauncher.getMotorVelocityInRPM()
                                    >= rightVelocity.getAsDouble() - 50) {
                        if (DriverStation.isTeleopEnabled()) {
                            setIsAtSpeed();
                        }
                    }
                },
                (b) -> {
                    // if (DriverStation.isTeleopEnabled()) {
                    setNotAtSpeed();
                    // }
                },
                () -> false);
    }

    public static void setIsAtSpeed() {
        isAtSpeed = true;
    }

    public static void setNotAtSpeed() {
        isAtSpeed = false;
    }

    /* Launcher Specific Commands */

    public static Command stopLeftMotor() {
        return leftLauncher.runStop().withName("LeftLauncher.stopMotor");
    }

    public static Command stopRightMotor() {
        return rightLauncher.runStop().withName("RightLauncher.stopMotor");
    }
}
