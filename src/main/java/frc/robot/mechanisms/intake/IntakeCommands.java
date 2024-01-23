package frc.robot.mechanisms.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakeCommands {
    private static Intake intake = Robot.intake;

    public static void setupDefaultCommand() {
        intake.setDefaultCommand(stopMotor().withName("Intake.default"));
    }

    public static Command runFull() {
        return intake.runVelocity(intake.config.maxSpeed).withName("Intake.runFull");
    }

    public static Command runTestin(){
        return intake.runPercentage(intake.config.testIntakePercentage)
                .withName("Intake.testIntake");
    }

    public static Command slowIntake() {
        return intake.runPercentage(intake.config.slowIntakePercentage)
                .withName("Intake.slowIntake");
    }

    public static Command intake() {
        return intake.runVelocity(intake.config.intake).withName("Intake.intake");
    }

    public static Command eject() {
        return intake.runPercentage(intake.config.ejectPercentage).withName("Intake.eject");
    }

    public static Command stopMotor() {
        return intake.runStop().withName("Intake.stopMotor");
    }
}
