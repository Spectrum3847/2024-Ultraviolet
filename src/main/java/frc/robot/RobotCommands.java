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

    //     public static Command laserCanFeed3() {
    //         return IntakeCommands.intake()
    //                 .alongWith(AmpTrapCommands.slowIntake(), FeederCommands.slowFeed())
    //                 .until(() -> Robot.feeder.hasNote())
    //                 .andThen(
    //                         AmpTrapCommands.slowEject()
    //                                 .alongWith(FeederCommands.slowFeedReverse())
    //                                 .until(() -> !Robot.feeder.hasNote()))
    //                 .andThen(
    //                         FeederCommands.slowFeed()
    //                                 .alongWith(AmpTrapCommands.slowIntake())
    //                                 .until(() -> Robot.feeder.hasNote()));
    //     }

    //     public static Command laserCanFeed2() {
    //         return IntakeCommands.intake()
    //                 .alongWith(AmpTrapCommands.fastIntake(), FeederCommands.fastFeed())
    //                 .until(() -> Robot.feeder.hasNote())
    //                 .andThen(
    //                         AmpTrapCommands.slowEject()
    //                                 .alongWith(FeederCommands.slowFeedReverse())
    //                                 .until(() -> !Robot.feeder.hasNote()))
    //                 .andThen(
    //                         FeederCommands.slowFeed()
    //                                 .alongWith(AmpTrapCommands.slowIntake())
    //                                 .until(() -> Robot.feeder.hasNote()));
    //     }

    public static Command laserCanFeed() {
        return IntakeCommands.intake().alongWith(AmpTrapCommands.slowIntake())
        // .until(() -> Robot.feeder.midNote())
        ;
    }

    public static Command dummyIntake() {
        return IntakeCommands.intake()
                .until(() -> Robot.feeder.getMotorVelocity() > 0)
                .andThen(
                        PilotCommands.rumble(1, 0.5)
                                // FeederCommands.addFeedRevolutions()
                                .withTimeout(0.5));
        // ); // add a .until to the feed revolutions later
    }

    public static Command onDemandLaunching() {
        return LauncherCommands.runOnDemandVelocity()
                // .alongWith(PivotCommands.onDemandPivot())
                .withName("RobotCommands.onDemandLaunching");
    }

    public static Command feedToAmp() {
        return FeederCommands.launchEject()
                .withTimeout(0.3)
                .andThen(FeederCommands.feedToAmp().alongWith(AmpTrapCommands.score()))
                .withName("RobotCommands.feedToAmp");
        // return
        // .until(Robot.ampTrap.lasercan::midNote)
        // .andThen(ElevatorCommands.amp().withTimeout(1))
        // .withName("RobotCommands.feedToAmp");
    }

    public static Command eject() {
        return FeederCommands.eject()
                .alongWith(AmpTrapCommands.eject(), IntakeCommands.eject())
                .withName("RobotCommands.eject");
    }

    // public static Command score() {
    //     return new ConditionalCommand(
    //             FeederCommands.launchEject(),
    //             AmpTrapCommands.score(),
    //             () -> Robot.elevator.getMotorPosition() < 5);
    // }

    public static Command score() {
        return new ConditionalCommand(
                FeederCommands.launchEject(),
                RobotCommands.feedToAmp(),
                () -> Robot.elevator.getMotorPosition() < 15);
    }

    public static Command intake() {
        return IntakeCommands.intake()
                .alongWith(AmpTrapCommands.intake(), FeederCommands.intake())
                .withName("RobotCommands.intake");
    }

    public static Command launchEject() {
        return AmpTrapCommands.launchEject()
                .alongWith(FeederCommands.launchEject())
                .withName("RobotCommands.launchEject");
    }

    public static Command coastModeMechanisms() {
        return AmpTrapCommands.coastMode()
                .alongWith(
                        ClimberCommands.coastMode(),
                        ElevatorCommands.coastMode(),
                        FeederCommands.coastMode(),
                        IntakeCommands.coastMode(),
                        LauncherCommands.coastMode(),
                        PivotCommands.coastMode())
                .withName("RobotCommands.coastModeMechanisms");
    }

    public static Command subwooferReady() {
        return LauncherCommands.runOnDemandVelocity()
                .alongWith(PivotCommands.subwoofer())
                .withName("RobotCommands.subwooferReady");
    }
}
