package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public static Command IntakeWithMotorSensor() {
        return IntakeCommands.intake()
                .alongWith(FeederCommands.feeder())
                .until(() -> Robot.feeder.getMotorVelocity() > 0.01)
                .andThen(PilotCommands.rumble(1, 0.5))
                .withTimeout(0.25);
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
                        FeederCommands.launchEject()
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
        return FeederCommands.launchEject()
                .withTimeout(0.1)
                .andThen(FeederCommands.feedToAmp())
                .alongWith(AmpTrapCommands.ampReady())
                .until(() -> Robot.ampTrap.hasNote())
                .andThen(
                        AmpTrapCommands.stopMotor()
                                .alongWith(FeederCommands.stopMotor(), ElevatorCommands.amp()));
    }

    public static Command ampReady8515() {
        return PivotCommands.ampScore()
                .alongWith(LauncherCommands.runAmpVelocity())
                .withName("RobotCommands.ampReady8515");
    }

    public static Command eject() {
        return FeederCommands.eject()
                .alongWith(AmpTrapCommands.eject(), IntakeCommands.eject())
                .withName("RobotCommands.eject");
    }

    public static Command home() {
        return PivotCommands.home();
    }

    public static Command climb() {
        return PivotCommands.climb();
    }

    // public static Command score() {
    //     return new ConditionalCommand(
    //             FeederCommands.launchEject(),
    //             AmpTrapCommands.score(),
    //             () -> Robot.elevator.getMotorPosition() < 5);
    // }

    public static Command score() {
        return FeederCommands.launchEject().alongWith(AmpTrapCommands.score());
        // return new ConditionalCommand(
        //         FeederCommands.launchEject(),
        //         RobotCommands.feedToAmp(),
        //         () -> Robot.elevator.getMotorPosition() < 15);
    }

    /*
     * Intake when called will check if the supply current is above 60 amps to eject and will continue to intake if not
     */
    public static Command intake() {
        return IntakeCommands.intake()
                .until(() -> (IntakeCommands.getSupplyCurrent() >= 60))
                .andThen(IntakeCommands.eject())
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
                .alongWith(PivotCommands.subwoofer()) // 25
                // rumble on ready
                // .until(
                //         () ->
                //                 (Robot.leftLauncher.getMotorVelocity() >= 4500
                //                         || Robot.rightLauncher.getMotorVelocity() >= 4500))
                // .andThen(
                //         PilotCommands.rumble(2, 1)
                //                 .alongWith(
                //                         LauncherCommands.runOnDemandVelocity(),
                //                         PivotCommands.subwoofer()))
                .withName("RobotCommands.subwooferReady");
    }

    public static Command podiumReady() {
        return LauncherCommands.runOnDemandVelocity()
                .alongWith(PivotCommands.onDemandPivot()) // 35
                .withName("RobotCommands.podium");
    }

    public static Command intake8515() {
        // return IntakeCommands.intake().onlyWhile(null).andThen();
        return IntakeCommands.intake()
                .until(() -> (Robot.feeder.getMotorVelocity() > 0))
                .andThen(FeederCommands.feeder().withTimeout(0.3));
        // .alongWith(IntakeCommands.stopMotor())
    }

    public static Command visionSpeakerLaunch() {
        return LauncherCommands.distanceVelocity(() -> Robot.vision.getDistanceToSpeaker())
                .alongWith(
                    PivotCommands.setPivotOnDistance(() -> Robot.vision.getDistanceToSpeaker())
                )
                .withName("RobotCommands.visionLaunch");
    }

    // // score speaker if in range, otherwise launch to feed
    // public static Command visionLaunch() {
    //     if (Field.isBlue()) {
    //         return Robot.swerve.getPose().getTranslation().getX() <= (Field.fieldLength / 2) - 1;
    //     } else {
    //         return Robot.swerve.getPose().getTranslation().getX() >= (Field.fieldLength / 2) + 1;
    //     }
    // }
}
