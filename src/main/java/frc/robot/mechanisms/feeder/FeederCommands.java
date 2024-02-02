package frc.robot.mechanisms.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class FeederCommands {
    private static Feeder feeder = Robot.feeder;

    public static void setupDefaultCommand() {
        feeder.setDefaultCommand(stopMotor().withName("Feeder.default"));
    }

    public static Command runFull() {
        return feeder.runVelocity(feeder.config.maxSpeed).withName("Feeder.runFull");
    }

    public static Command slowFeed() {
        return feeder.runPercentage(feeder.config.slowFeederPercentage).withName("Feeder.slowFeed");
    }

    public static Command feeder() {
        return feeder.runVelocity(feeder.config.feed).withName("Feeder.feed");
    }

    public static Command eject() {
        return feeder.runVelocity(feeder.config.eject).withName("Feeder.eject");
    }

    public static Command launchEject() {
        return feeder.runVelocity(feeder.config.launchEject).withName("Feeder.launchEject");
    }

    public static Command intake() {
        return feeder.runVelocity(feeder.config.intake).withName("Feeder.intake");
    }

    public static Command stopMotor() {
        return feeder.runStop().withName("Feeder.stopMotor");
    }

    public static Command testForward() {
        return feeder.runPercentage(feeder.config.testForwardPercent);
    }

    public static Command testBack() {
        return feeder.runPercentage(feeder.config.testBackPercent);
    }

    public static Command feedToAmp() {
        return feeder.runVelocity(feeder.config.feedToAmp).withName("Feeder.feedToAmp");
    }

    public static Command coastMode() {
        return feeder.coastMode();
    }
}
