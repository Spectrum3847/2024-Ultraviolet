package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakeCommands {
    private static Intake intake = Robot.intake;

    public static void setupDefaultCommand() {
        intake.setDefaultCommand(holdPercentOutput().withName("Intake.default"));
    }

    public static Command runFull() {
        return intake.runVelocity(intake.config.maxSpeed).withName("Intake.runFull");
    }

    public static Command slowIntake() {
        return intake.runPercentage(intake.config.slowIntakePercentage)
                .withName("Intake.slowIntake");
    }

    public static Command holdPercentOutput() {
        return intake.runPercentage(intake.config.holdIntakePercentage)
                .withName("Intake.holdPercentOutput");
    }

    public static Command intake() {
        return intake.runVelocity(intake.config.intake).withName("Intake.intake");
    }

    public static Command eject() {
        return intake.runVelocity(intake.config.eject).withName("Intake.eject");
    }

    public static Command drop() {
        return intake.runVelocity(intake.config.drop).withName("Intake.drop");
    }

    public static Command floorDrop() {
        return intake.runVelocity(intake.config.floorDrop).withName("Intake.floorDrop");
    }

    public static Command stopMotor() {
        return intake.runStop().withName("Intake.stopMotor");
    }
}
