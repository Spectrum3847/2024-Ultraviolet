// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.mechanisms.amptrap.AmpTrap;
import frc.robot.mechanisms.intake.Intake;
import frc.spectrumLib.util.Conversions;

public class LaserCanFeed extends Command {
    private boolean passedZero;
    private int setPoint; // millimeters
    private boolean continueFeeding;
    private Feeder feeder = Robot.feeder;
    private AmpTrap ampTrap = Robot.ampTrap;
    private Intake intake = Robot.intake;

    /** Creates a new LaserCanFeed. */
    public LaserCanFeed(int setPoint) {
        this.setPoint = setPoint;
        passedZero = false;
        continueFeeding = false;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(feeder, ampTrap, intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        passedZero = false;
        continueFeeding = true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        int distance = Robot.feeder.getLaserCanDistance();

        if(distance < setPoint) {
            intake.stop();
        }

        if (Math.abs(distance) - 10 <= 0) {
            passedZero = true;
        }

        if (passedZero && distance >= setPoint) {
            continueFeeding = false;
        }

        if (continueFeeding) {
            feeder.setVelocity(Conversions.RPMtoRPS(feeder.config.testFeed));
            ampTrap.setVelocity(Conversions.RPMtoRPS(feeder.config.testFeed));
            intake.setVelocity(Conversions.RPMtoRPS(intake.config.intake));
        } else {
            feeder.stop();
            ampTrap.stop();
            intake.stop();
        }
        System.out.println(distance);
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
