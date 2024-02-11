package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.mechanisms.amptrap.AmpTrapCommands;
import frc.robot.mechanisms.climber.ClimberCommands;
import frc.robot.mechanisms.elevator.ElevatorCommands;
import frc.robot.mechanisms.feeder.FeederCommands;
import frc.robot.mechanisms.intake.IntakeCommands;
import frc.robot.mechanisms.launcher.LauncherCommands;
import frc.robot.mechanisms.pivot.PivotCommands;

/**
 * This class is used for commands that use multiple subsystems and don't directly call a gamepad.
 * This is often command groups such as moving an arm and turning on an intake, etc. In 2023 we
 * called this MechanismCommands.java
 */
public class RobotCommands {

    public static Command laserCanFeed() {
        return IntakeCommands.intake()
                .alongWith(AmpTrapCommands.slowIntake(), FeederCommands.slowFeed())
                .until(() -> Robot.feeder.hasNote())
                .andThen(
                        AmpTrapCommands.slowIntake()
                                .alongWith(FeederCommands.slowFeed(), IntakeCommands.stopMotor())
                                .until(() -> !Robot.feeder.hasNote()))
                .andThen(
                        FeederCommands.slowFeedReverse()
                                .alongWith(AmpTrapCommands.stopMotor())
                                .until(() -> Robot.feeder.hasNote()));
    }

    public static Command onDemandLaunching() {
        return LauncherCommands.runOnDemandVelocity()
                .alongWith(PivotCommands.onDemandPivot())
                .withName("RobotCommands.onDemandLaunching");
    }

    public static Command feedToAmp() {
        return FeederCommands.feedToAmp()
                .alongWith(AmpTrapCommands.score())
                .withName("RobotCommands.feedToAmp");
    }

    public static Command eject() {
        return FeederCommands.eject()
                .alongWith(AmpTrapCommands.eject(), IntakeCommands.eject())
                .withName("RobotCommands.eject");
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
        return PivotCommands.subwoofer()
                .alongWith(LauncherCommands.subwoofer())
                .withName("RobotCommands.subwooferReady");
    }

    public static Command launchReady() {
        return PivotCommands.autoLaunch()
                .alongWith(LauncherCommands.subwoofer())
                .withName("RobotCommands.subwooferReady");
    }
}
