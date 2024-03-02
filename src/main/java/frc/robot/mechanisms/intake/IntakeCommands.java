package frc.robot.mechanisms.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakeCommands {
    private static Intake intake = Robot.intake;

    public static void setupDefaultCommand() {
        intake.setDefaultCommand(stopMotor().withName("Intake.default"));
    }

    public static Command runFull() {
        return intake.runVelocityTorqueCurrentFOC(intake.config.maxSpeed)
                .withName("Intake.runFull");
    }

    public static Command runTestin() {
        return intake.runPercentage(intake.config.testIntakePercentage)
                .withName("Intake.testIntake");
    }

    public static Command runVelocityTestin() {
        return intake.runVelocityTorqueCurrentFOC(intake.config.testVelocity)
                .withName("Intake.testVelocity");
    }

    public static Command slowIntake() {
        return intake.runPercentage(intake.config.slowIntakePercentage)
                .withName("Intake.slowIntake");
    }

    public static Command intake() {
        return intake.runVelocityTorqueCurrentFOC(intake.config.intake).withName("Intake.intake");
    }

    public static Command eject() {
        return intake.runVelocityTorqueCurrentFOC(intake.config.eject).withName("Intake.eject");
    }

    public static Command coastMode() {
        return intake.coastMode();
    }

    public static Command stopMotor() {
        return intake.runStop().withTimeout(0.01).withName("Intake.stopMotor");
    }
}
