// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.swerve.Swerve;

public class RotationControllerUpdate extends Command {
    double kP, kI, kD;
    Swerve swerve = Robot.swerve;

    public RotationControllerUpdate() {
        this.kP = swerve.config.kPRotationController;
        this.kI = swerve.config.kIRotationController;
        this.kD = swerve.config.kDRotationController;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        kP = SmartDashboard.getNumber("rotationControllerkP", kP);
        kI = SmartDashboard.getNumber("rotationControllerkI", kI);
        kD = SmartDashboard.getNumber("rotationControllerkD", kD);

        Robot.swerve.updateRotationControllerPID(kP, kI, kD, 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
