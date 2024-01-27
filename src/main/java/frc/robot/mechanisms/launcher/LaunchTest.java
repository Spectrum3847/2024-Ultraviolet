// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms.launcher;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.spectrumLib.util.Conversions;

public class LaunchTest extends Command {
    double leftLaunchSpeed = 0;
    double rightLaunchSpeed = 0;

    double defaultValue = 0;
    /** Creates a new CubeLaunchTest. */
    public LaunchTest(double configValue) {
        defaultValue = configValue;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.leftLauncher, Robot.rightLauncher);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        leftLaunchSpeed = SmartDashboard.getNumber("leftLaunchSpeed", 0);
        rightLaunchSpeed = SmartDashboard.getNumber("rightLaunchSpeed", 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Robot.leftLauncher.setVelocity(Conversions.RPMtoRPS(leftLaunchSpeed));
        Robot.rightLauncher.setVelocity(Conversions.RPMtoRPS(rightLaunchSpeed));
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
