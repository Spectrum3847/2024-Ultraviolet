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
    boolean rumblePilot;
    /** Creates a new CubeLaunchTest. */
    public LaunchTest(double configValue) {
        defaultValue = configValue;
        rumblePilot = false;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.leftLauncher, Robot.rightLauncher);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        leftLaunchSpeed = SmartDashboard.getNumber("leftLaunchSpeed", defaultValue);
        rightLaunchSpeed = SmartDashboard.getNumber("rightLaunchSpeed", defaultValue);
        rumblePilot = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Robot.leftLauncher.setVelocityTorqueCurrentFOC(Conversions.RPMtoRPS(leftLaunchSpeed));
        Robot.rightLauncher.setVelocityTorqueCurrentFOC(Conversions.RPMtoRPS(rightLaunchSpeed));

        if (!rumblePilot
                && ((Robot.leftLauncher.getMotorVelocityInRPM() >= leftLaunchSpeed - 100)
                        || (Robot.rightLauncher.getMotorVelocityInRPM()
                                >= rightLaunchSpeed - 100))) {
            rumblePilot = true;
            Robot.pilot.rumbleCommand(0.5, 1).withName("LaunchRumble").schedule();
        }
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
