package frc.robot.mechanisms.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakeCommands {
    private static Intake intake = Robot.intake;

    public static void setupDefaultCommand() {
        intake.setDefaultCommand(stopMotor().ignoringDisable(true).withName("Intake.default"));
    }

    public static Command runFull() {
        return intake.runVelocityTorqueCurrentFOC(intake.config.maxSpeed)
                .withName("Intake.runFull");
    }

    public static Command intake() {
        return intake.runVelocityTorqueCurrentFOC(intake.config.intake).withName("Intake.intake");
    }

    public static Command slowIntake() {
        return intake.runVelocityTorqueCurrentFOC(intake.config.slowIntake)
                .withName("Intake.slowIntake");
    }

    public static Command eject() {
        return intake.runVelocityTorqueCurrentFOC(intake.config.eject).withName("Intake.eject");
    }

    public static Command coastMode() {
        return intake.coastMode();
    }

    public static Command intakeWithoutCurrentLimit() {
        return intake.intakeWithoutCurrentLimit();
    }

    public static Command stopMotor() {
        return intake.runStop().withName("Intake.stopMotor");
    }

    public static Command ensureBrakeMode() {
        return intake.ensureBrakeMode();
    }
}
