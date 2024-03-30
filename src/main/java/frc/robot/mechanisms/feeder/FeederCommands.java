package frc.robot.mechanisms.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class FeederCommands {
    private static Feeder feeder = Robot.feeder;

    public static void setupDefaultCommand() {
        feeder.setDefaultCommand(stopMotor().ignoringDisable(true).withName("Feeder.default"));
    }

    public static Command runFull() {
        return feeder.runVelocity(feeder.config.maxSpeed).withName("Feeder.runFull");
    }

    public static Command addFeedRevolutions() {
        return feeder.runAddPosition(feeder.config.addedFeedRotations)
                .withName("Feeder.addFeedRevolutions");
    }

    public static Command eject() {
        return feeder.runVelocity(feeder.config.eject).withName("Feeder.eject");
    }

    public static Command score() {
        return feeder.runVelocity(feeder.config.score).withName("Feeder.score");
    }

    public static Command autoFeed() {
        return feeder.runVelocity(feeder.config.autoFeed).withName("Feeder.score");
    }

    public static Command slowFeed() {
        return feeder.runVelocity(feeder.config.slowFeed).withName("Feeder.slowFeed");
    }

    public static Command slowEject() {
        return feeder.runVelocity(feeder.config.slowEject).withName("Feeder.slowEject");
    }

    public static Command intake() {
        return feeder.runVelocity(feeder.config.intake).withName("Feeder.intake");
    }

    public static Command stopMotor() {
        return feeder.runStop().withName("Feeder.stopMotor");
    }

    public static Command feedToAmp() {
        return feeder.runVelocity(feeder.config.feedToAmp).withName("Feeder.feedToAmp");
    }

    public static Command launchEject() {
        return feeder.runVelocity(feeder.config.launchEject).withName("Feeder.launchEject");
    }

    public static Command manualSource() {
        return feeder.runVelocity(feeder.config.manualSource).withName("Feeder.manualSource");
    }

    public static Command coastMode() {
        return feeder.coastMode();
    }

    public static Command ensureBrakeMode() {
        return feeder.ensureBrakeMode();
    }
}
