package frc.robot;

import frc.robot.mechanisms.amptrap.AmpTrapCommands;
import frc.robot.mechanisms.climber.ClimberCommands;
import frc.robot.mechanisms.elevator.ElevatorCommands;
import frc.robot.mechanisms.feeder.FeederCommands;
import frc.robot.mechanisms.intake.IntakeCommands;
import frc.robot.mechanisms.launcher.LauncherCommands;
import frc.robot.mechanisms.pivot.PivotCommands;

public class Coordinator {
    
    public void update() {}

    public void applyRobotState(State state) {
        switch (state) {
            case REHOME -> {
                AmpTrapCommands.coastMode();
                ClimberCommands.coastMode();
                ElevatorCommands.home();
                FeederCommands.stopMotor();
                IntakeCommands.stopMotor();
                LauncherCommands.stopMotors();
                PivotCommands.intake();
            }
            case PRE_SHOT -> {
                AmpTrapCommands.coastMode();
                ClimberCommands.coastMode();
                ElevatorCommands.holdPosition();
                FeederCommands.stopMotor();
                IntakeCommands.stopMotor();
                // handles preping the launcher, pivot, and drivetrain
                RobotCommands.visionSpeakerLaunch();
            }
            case SHOT -> {
                AmpTrapCommands.score();
                ClimberCommands.coastMode();
                ElevatorCommands.holdPosition();
                FeederCommands.launchEject();
                IntakeCommands.slowIntake();
                // handles running the launcher, pivot, and drivetrain
                RobotCommands.visionSpeakerLaunch();
            }
            case PRE_AMP -> {
                ClimberCommands.coastMode();
                IntakeCommands.stopMotor();
                LauncherCommands.stopMotors();
                PivotCommands.intake();
                // hands preping the amp trap, elevator, and feeder
                RobotCommands.amp();
            }
            case AMP -> {
                AmpTrapCommands.score();
                ClimberCommands.coastMode();
                ElevatorCommands.holdPosition();
                FeederCommands.score();
                IntakeCommands.intake();
                LauncherCommands.stopMotors();
                PivotCommands.intake();
            }
            case INTAKE -> {
                ClimberCommands.coastMode();
                FeederCommands.stopMotor();
                LauncherCommands.stopMotors();
                // handles running the intake, amp trap, elevator, and pivot
                RobotCommands.smartIntake();
            }
            case EJECT -> {
                AmpTrapCommands.eject();
                ClimberCommands.coastMode();
                ElevatorCommands.home();
                FeederCommands.eject();
                IntakeCommands.eject();
                LauncherCommands.eject();
                PivotCommands.intake();
            }
            default -> {

            }
        }
    }

}