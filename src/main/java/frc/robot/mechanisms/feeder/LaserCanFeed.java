// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class LaserCanFeed extends Command {
  private boolean setPointPassedOnce;
  private int setPoint; //millimeters
  private int tolerance; //millimeterse
  private int count;
  private int thresholdCount;
  private boolean continueFeeding;
  /** Creates a new LaserCanFeed. */
  public LaserCanFeed(int setPoint, int tolerance) {
    this.setPoint = setPoint;
    this.tolerance = tolerance;
    setPointPassedOnce = false;
    count = 0;
    thresholdCount = 0;
    continueFeeding = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.feeder, Robot.ampTrap);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setPointPassedOnce = false;
    count = 0;
    thresholdCount = 0;
    continueFeeding = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!setPointPassedOnce) {
      if(Math.abs(Robot.feeder.getLaserCanDistance() - setPoint) <= tolerance) {
        setPointPassedOnce = true;
      }
    } else {
      if(Robot.feeder.getLaserCanDistance() - setPoint >= tolerance) {
        if(thresholdCount >= 4) {
          count++;
        }
        thresholdCount++;
      }
    }

    if(count >= 1) {
      continueFeeding = false;
    }

    if(continueFeeding) {
      
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
