package frc.robot.mechanisms.launcher;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Robot;
import frc.robot.leds.LEDsCommands;
import frc.robot.pilot.PilotCommands;
=======
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Robot;
import frc.robot.RobotTelemetry;
>>>>>>> Madtown-Auto
import java.util.function.DoubleSupplier;

public class LauncherCommands {
    private static LeftLauncher leftLauncher = Robot.leftLauncher;
    private static RightLauncher rightLauncher = Robot.rightLauncher;

<<<<<<< HEAD
    public static final InterpolatingDoubleTreeMap DISTANCE_MAP = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap FEED_DISTANCE_MAP =
            new InterpolatingDoubleTreeMap();

    static {
        // launching
        DISTANCE_MAP.put(1.18, 4500.0);
        DISTANCE_MAP.put(1.76, 4500.0);
        DISTANCE_MAP.put(2.79, 4500.0);
        DISTANCE_MAP.put(2.93, 4500.0);
=======
    public static boolean isAtSpeed = false;

    public static final InterpolatingDoubleTreeMap DISTANCE_MAP = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap FEED_DISTANCE_MAP =
            new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap DEEP_FEED_DISTANCE_MAP =
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
        // FEED_DISTANCE_MAP.put(6.0, 3300.0);
        // FEED_DISTANCE_MAP.put(6.08, 3200.0);
        // FEED_DISTANCE_MAP.put(6.47, 3200.0);
        // FEED_DISTANCE_MAP.put(6.96, 3200.0);
        // FEED_DISTANCE_MAP.put(7.54, 3700.0);
        // FEED_DISTANCE_MAP.put(7.74, 3800.0);
        // FEED_DISTANCE_MAP.put(9.05, 3950.0);
        FEED_DISTANCE_MAP.put(7.0, 4900.0);

        DEEP_FEED_DISTANCE_MAP.put(7.0, 5000.0);
>>>>>>> Madtown-Auto
    }

    public static void setupDefaultCommand() {
        leftLauncher.setDefaultCommand(
<<<<<<< HEAD
                stopLeftMotor().withName("LeftLauncher.default").ignoringDisable(true));
        rightLauncher.setDefaultCommand(
                stopRightMotor().withName("RightLauncher.default").ignoringDisable(true));
=======
                stopLeftMotor().ignoringDisable(true).withName("LeftLauncher.default"));
        rightLauncher.setDefaultCommand(
                stopRightMotor().ignoringDisable(true).withName("RightLauncher.default"));
>>>>>>> Madtown-Auto
    }

    /* Launch Commands */

    public static DoubleSupplier getRPMfromDistance(DoubleSupplier distance) {
<<<<<<< HEAD
        return () -> DISTANCE_MAP.get(distance.getAsDouble());
    }

    public static Command runPercentOutput(double output) {
        return leftLauncher.runPercentage(output).alongWith(rightLauncher.runPercentage(output));
=======
        return () -> getMapRPM(DISTANCE_MAP, distance.getAsDouble());
        // return () -> DISTANCE_MAP.get(distance.getAsDouble());
    }

    public static DoubleSupplier getRPMFromFeedDistance(DoubleSupplier distance) {
        return () -> getMapRPM(FEED_DISTANCE_MAP, distance.getAsDouble());
        // return () -> FEED_DISTANCE_MAP.get(distance.getAsDouble());
    }

    public static DoubleSupplier getRPMFromDeepFeedDistance(DoubleSupplier distance) {
        return () -> getMapRPM(DEEP_FEED_DISTANCE_MAP, distance.getAsDouble());
        // return () -> FEED_DISTANCE_MAP.get(distance.getAsDouble());
>>>>>>> Madtown-Auto
    }

    public static Command distanceVelocity(DoubleSupplier distance) {
        return velocityTCFOCrpm(getRPMfromDistance(distance));
    }

<<<<<<< HEAD
=======
    public static Command feedDistanceVelocity(DoubleSupplier distance) {
        return velocityTCFOCrpm(getRPMFromFeedDistance(distance));
    }

    public static Command deepFeedDistanceVelocity(DoubleSupplier distance) {
        return velocityTCFOCrpm(getRPMFromDeepFeedDistance(distance));
    }

>>>>>>> Madtown-Auto
    public static Command velocityTCFOCrpm(DoubleSupplier velocityRPM) {
        return leftLauncher
                .runVelocityTCFOCrpm(velocityRPM)
                .alongWith(
                        rightLauncher.runVelocityTCFOCrpm(velocityRPM),
<<<<<<< HEAD
                        sendLauncherFeedback(velocityRPM.getAsDouble(), velocityRPM.getAsDouble()));
=======
                        sendLauncherFeedback(velocityRPM));
>>>>>>> Madtown-Auto
    }

    public static Command runOnDemandVelocity() {
        return new OnDemandLaunch(leftLauncher.config.testVelocity)
                .withName("Launcher.onDemandVelocity");
    }

<<<<<<< HEAD
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

    public static Command eject() {
        return runLauncherPercentages(-.4, -.4).withName("Launcher.ejectSequence");
    }

=======
>>>>>>> Madtown-Auto
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
<<<<<<< HEAD
        return runPercentOutput(1).withName("Launcher.subwoofer");
    }

    public static Command launchReadyPreload() {
        return runLauncherVelocities(
                leftLauncher.config.launchReadyPreload, rightLauncher.config.launchReadyPreload);
=======
        return runTorqueLauncherVelocities(
                        leftLauncher.config.subwoofer, rightLauncher.config.subwoofer)
                .withName("Launcher.subwoofer");
>>>>>>> Madtown-Auto
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
<<<<<<< HEAD
                .runVelocityTorqueCurrentFOC(leftVelocity)
                .alongWith(rightLauncher.runVelocityTorqueCurrentFOC(rightVelocity))
=======
                .runVelocity(leftVelocity)
                .alongWith(
                        rightLauncher.runVelocity(rightVelocity),
                        sendLauncherFeedback(leftVelocity, rightVelocity))
>>>>>>> Madtown-Auto
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

<<<<<<< HEAD
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
=======
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
>>>>>>> Madtown-Auto
                },
                () -> false);
    }

<<<<<<< HEAD
=======
    public static void setIsAtSpeed() {
        isAtSpeed = true;
    }

    public static void setNotAtSpeed() {
        isAtSpeed = false;
    }

>>>>>>> Madtown-Auto
    /* Launcher Specific Commands */

    public static Command stopLeftMotor() {
        return leftLauncher.runStop().withName("LeftLauncher.stopMotor");
    }

    public static Command stopRightMotor() {
        return rightLauncher.runStop().withName("RightLauncher.stopMotor");
    }
}
