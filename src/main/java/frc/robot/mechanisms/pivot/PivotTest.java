// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class PivotTest extends Command {
    double pivotAngle = 0;

    double defaultValue = 0;
    /** Creates a new CubePivotTest. */
    public PivotTest(double configValue) {
        defaultValue = configValue;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.pivot);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pivotAngle = SmartDashboard.getNumber("pivotAngle", defaultValue);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Robot.pivot.setMMPosition(Robot.pivot.angleToRotation(pivotAngle));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
