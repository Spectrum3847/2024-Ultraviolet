package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.mechanisms.amptrap.AmpTrapCommands;
import frc.robot.mechanisms.climber.ClimberCommands;
import frc.robot.mechanisms.elevator.ElevatorCommands;
import frc.robot.mechanisms.feeder.FeederCommands;
import frc.robot.mechanisms.intake.IntakeCommands;
import frc.robot.mechanisms.launcher.LauncherCommands;
import frc.robot.mechanisms.pivot.PivotCommands;
import frc.robot.pilot.PilotCommands;
import frc.robot.swerve.commands.SwerveCommands;
import frc.robot.vision.VisionCommands;

/**
 * This class is used for commands that use multiple subsystems and don't directly call a gamepad.
 * This is often command groups such as moving an arm and turning on an intake, etc. In 2023 we
 * called this MechanismCommands.java
 */
public class RobotCommands {

    public static Command feed() {
        return new ConditionalCommand(
                new InstantCommand(), laserCanFeed(), Robot.feeder.lasercan::hasNote);
    }

    public static Command laserCanFeed() {
        return IntakeCommands.intake().alongWith(AmpTrapCommands.feed())
        // .until(() -> Robot.feeder.midNote())
        ;
    }

    public static Command intakeWithMotorSensor() {
        return IntakeCommands.intake()
                .until(() -> Robot.feeder.getMotorVelocity() > 0.01)
                .andThen(PilotCommands.rumble(1, 0.5).alongWith(VisionCommands.blinkLimelights()));
        // FeederCommands.addFeedRevolutions().withTimeout(0.15 )));
        // ); // add a .until to the feed revolutions later
    }

    public static Command onDemandLaunching() {
        return LauncherCommands.runOnDemandVelocity()
                // .alongWith(PivotCommands.onDemandPivot())
                .withName("RobotCommands.onDemandLaunching");
    }

    public static Command feedToAmp() {
        return AmpTrapCommands.score()
                .alongWith(
                        FeederCommands.score()
                                .withTimeout(0.15)
                                .andThen(FeederCommands.feedToAmp()))
                .withName("RobotCommands.feedToAmp");

        /* With lasercan */
        // return FeederCommands.launchEject()
        //         .withTimeout(0.15)
        //         .andThen(
        //                 FeederCommands.feedToAmp()
        //                         .alongWith(AmpTrapCommands.score())
        //                         .until(Robot.ampTrap.lasercan::bigMidNote),
        // ElevatorCommands.amp().withTimeout(1))
        //         .withName("RobotCommands.feedToAmp");
    }

    public static Command ampReady() {
        return FeederCommands.score()
                .withTimeout(0.1)
                .andThen(FeederCommands.feedToAmp())
                .alongWith(AmpTrapCommands.ampReady())
                .until(() -> Robot.ampTrap.hasNote())
                .andThen(
                        AmpTrapCommands.stopMotor()
                                .alongWith(FeederCommands.stopMotor(), ElevatorCommands.amp()));
    }

    public static Command eject() {
        return FeederCommands.eject()
                .alongWith(
                        AmpTrapCommands.eject(), IntakeCommands.eject(), LauncherCommands.eject())
                .withName("RobotCommands.eject");
    }

    // public static Command score() {
    //     return new ConditionalCommand(
    //             FeederCommands.launchEject(),
    //             AmpTrapCommands.score(),
    //             () -> Robot.elevator.getMotorPosition() < 5);
    // }

    public static Command score() {
        return FeederCommands.score().alongWith(AmpTrapCommands.score());
        // return new ConditionalCommand(
        //         FeederCommands.launchEject(),
        //         RobotCommands.feedToAmp(),
        //         () -> Robot.elevator.getMotorPosition() < 15);
    }

    public static Command intake() {
        return IntakeCommands.intake()
                .alongWith(AmpTrapCommands.intake(), FeederCommands.intake())
                .withName("RobotCommands.intake");
    }

    public static Command coastModeMechanisms() {
        return AmpTrapCommands.coastMode()
                .alongWith(
                        ClimberCommands.coastMode(),
                        ElevatorCommands.coastMode(),
                        FeederCommands.coastMode(),
                        IntakeCommands.coastMode(),
                        LauncherCommands.coastMode(),
                        PivotCommands.coastMode(),
                        SwerveCommands.coastMode())
                .withName("RobotCommands.coastModeMechanisms");
    }

    public static Command subwooferReady() {
        return LauncherCommands.runOnDemandVelocity()
                .alongWith(PivotCommands.subwoofer())
                .withName("RobotCommands.subwooferReady");
    }

    public static Command podiumReady() {
        return LauncherCommands.deepShot()
                .alongWith(PivotCommands.podium(), PilotCommands.podiumAimingDrive())
                .withName("RobotCommands.podium");
    }

    public static Command AmpWingReady() {
        return LauncherCommands.deepShot()
                .alongWith(PivotCommands.ampWing(), PilotCommands.ampWingAimingDrive())
                .withName("RobotCommands.ampWing");
    }

    public static Command topClimb() {
        return ClimberCommands.topClimb().alongWith(PivotCommands.home());
    }
}
