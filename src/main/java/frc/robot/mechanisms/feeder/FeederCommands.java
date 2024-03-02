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

    public static Command intake() {
        return feeder.runVelocity(feeder.config.intake).withName("Feeder.intake");
    }

    public static Command stopMotor() {
        return feeder.runStop().withName("Feeder.stopMotor");
    }

    public static Command feedToAmp() {
        return feeder.runVelocity(feeder.config.feedToAmp).withName("Feeder.feedToAmp");
    }

    public static Command coastMode() {
        return feeder.coastMode();
    }
}
