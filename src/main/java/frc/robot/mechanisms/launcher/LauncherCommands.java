package frc.robot.mechanisms.launcher;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Robot;
import frc.robot.RobotTelemetry;
import java.util.function.DoubleSupplier;

public class LauncherCommands {
    private static LeftLauncher leftLauncher = Robot.leftLauncher;
    private static RightLauncher rightLauncher = Robot.rightLauncher;

    public static boolean isAtSpeed = false;

    public static final InterpolatingDoubleTreeMap DISTANCE_MAP = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap FEED_DISTANCE_MAP =
            new InterpolatingDoubleTreeMap();

    static {
        /* Old Launching */
        // DISTANCE_MAP.put(1.505, 3500.0);
        // DISTANCE_MAP.put(2.629, 4500.0);
        // DISTANCE_MAP.put(3.969, 4500.0);
        // DISTANCE_MAP.put(4.269, 5200.0);
        // DISTANCE_MAP.put(4.899, 5200.0);
        // DISTANCE_MAP.put(5.189, 5200.0);
        // DISTANCE_MAP.put(5.829, 5200.0);
        // DISTANCE_MAP.put(6.229, 5200.0);

        /*Launching */
        // 4500 RPM shots
        DISTANCE_MAP.put(0.0, 4500.0);
        DISTANCE_MAP.put(4.1, 4500.0);
        // 5000 RPM shots
        DISTANCE_MAP.put(4.11, 5000.0);
        DISTANCE_MAP.put(5.9, 5000.0);

        // feed launching -- OLD
        // FEED_DISTANCE_MAP.put(6.0, 3000.0);
        // FEED_DISTANCE_MAP.put(6.08, 3000.0);
        // FEED_DISTANCE_MAP.put(6.47, 3000.0);
        // FEED_DISTANCE_MAP.put(6.96, 3000.0);
        // FEED_DISTANCE_MAP.put(7.54, 3000.0);
        // FEED_DISTANCE_MAP.put(7.74, 3250.0);
        // FEED_DISTANCE_MAP.put(9.05, 4000.0);

        // feed launching -- REVERT
        FEED_DISTANCE_MAP.put(6.0, 3300.0);
        FEED_DISTANCE_MAP.put(6.08, 3200.0);
        FEED_DISTANCE_MAP.put(6.47, 3200.0);
        FEED_DISTANCE_MAP.put(6.96, 3200.0);
        FEED_DISTANCE_MAP.put(7.54, 3700.0);
        FEED_DISTANCE_MAP.put(7.74, 3800.0);
        FEED_DISTANCE_MAP.put(9.05, 3950.0);
    }

    public static void setupDefaultCommand() {
        leftLauncher.setDefaultCommand(
                stopLeftMotor().ignoringDisable(true).withName("LeftLauncher.default"));
        rightLauncher.setDefaultCommand(
                stopRightMotor().ignoringDisable(true).withName("RightLauncher.default"));
    }

    /* Launch Commands */

    public static DoubleSupplier getRPMfromDistance(DoubleSupplier distance) {
        return () -> getMapRPM(DISTANCE_MAP, distance.getAsDouble());
        // return () -> DISTANCE_MAP.get(distance.getAsDouble());
    }

    public static DoubleSupplier getRPMFromFeedDistance(DoubleSupplier distance) {
        return () -> getMapRPM(FEED_DISTANCE_MAP, distance.getAsDouble());
        // return () -> FEED_DISTANCE_MAP.get(distance.getAsDouble());
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

    public static Command manualFeed() {
        return runLauncherVelocities(
                        leftLauncher.config.manualFeed, rightLauncher.config.manualFeed)
                .withName("Launcher.manualFeed");
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

    public static Command subwoofer() {
        return runTorqueLauncherVelocities(
                        leftLauncher.config.subwoofer, rightLauncher.config.subwoofer)
                .withName("Launcher.subwoofer");
    }

    public static Command autoShoot() {
        return runTorqueLauncherVelocities(
                        leftLauncher.config.autoShoot, rightLauncher.config.autoShoot)
                .withName("Launcher.subwoofer");
    }

    public static Command manualSource() {
        return runTorqueLauncherVelocities(
                        leftLauncher.config.manualSource, rightLauncher.config.manualSource)
                .withName("Launcher.manualSource");
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
