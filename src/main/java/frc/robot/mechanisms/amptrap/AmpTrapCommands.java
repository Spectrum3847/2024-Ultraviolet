package frc.robot.mechanisms.amptrap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class AmpTrapCommands {
    private static AmpTrap ampTrap = Robot.ampTrap;

    public static void setupDefaultCommand() {
        ampTrap.setDefaultCommand(stopMotor().withName("AmpTrap.default"));
    }

    public static Command runFull() {
        return ampTrap.runVelocity(ampTrap.config.maxSpeed).withName("AmpTrap.runFull");
    }

    public static Command slowIntake() {
        return ampTrap.runVelocity(ampTrap.config.slowIntake).withName("AmpTrap.slowIntake");
    }

    public static Command fastIntake() {
        return ampTrap.runVelocity(ampTrap.config.fastIntake).withName("AmpTrap.fastIntake");
    }

    public static Command slowEject() {
        return ampTrap.runVelocity(ampTrap.config.slowEject).withName("AmpTrap.slowEject");
    }

    public static Command intake() {
        return ampTrap.runVelocity(ampTrap.config.intake).withName("AmpTrap.intake");
    }

    public static Command score() {
        return ampTrap.runVelocity(ampTrap.config.score).withName("AmpTrap.score");
    }

    public static Command launchEject() {
        return ampTrap.runVelocity(ampTrap.config.launchEject).withName("AmpTrap.launchEject");
    }

    public static Command eject() {
        return ampTrap.runVelocity(ampTrap.config.eject).withName("AmpTrap.eject");
    }

    public static Command stopMotor() {
        return ampTrap.runStop().withName("AmpTrap.stopMotor");
    }

    public static Command testForward() {
        return ampTrap.runPercentage(ampTrap.config.testForwardPercent);
    }

    public static Command testReverse() {
        return ampTrap.runPercentage(ampTrap.config.testBackPercent);
    }

    public static Command coastMode() {
        return ampTrap.coastMode();
    }
}
