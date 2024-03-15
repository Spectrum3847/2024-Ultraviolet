package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.leds.LEDs;
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

    // Auto aim, set pivot, and launcher velocities
    public static Command visionLaunch() {
        return PilotCommands.aimToSpeaker()
                .alongWith(
                        LauncherCommands.distanceVelocity(() -> Robot.vision.getSpeakerDistance()),
                        PivotCommands.setPivotOnDistance(() -> Robot.vision.getSpeakerDistance()))
                .withName("RobotCommands.visionLaunch");
    }

    public static Command testMap() {
        return PilotCommands.aimToSpeaker().alongWith(RobotCommands.onDemandLaunching());
    }
    // Run the intake for at least 0.4 seconds OR until the lasercan sees the note,
    // (also get pivot
    // to a good position if it's
    // too low, and stop it when intaking is done), then rumble and blink
    public static Command smartIntake() {
        return ElevatorCommands.home()
                .alongWith(
                        IntakeCommands.intake().alongWith(AmpTrapCommands.intake())
                                .withTimeout(0.4)
                                .andThen(
                                        IntakeCommands.intake()
                                                .alongWith(AmpTrapCommands.intake())
                                                .until(Robot.feeder::intakedNote)
                                                .deadlineWith(
                                                        PivotCommands.intake()
                                                                .onlyIf(
                                                                        () ->
                                                                                Robot.pivot
                                                                                                .getMotorPercentAngle()
                                                                                        < Robot
                                                                                                .pivot
                                                                                                .config
                                                                                                .intake))
                                                .andThen(
                                                        IntakeCommands.stopMotor()
                                                                .alongWith(
                                                                        PilotCommands.rumble(1, 0.5)
                                                                                .alongWith(
                                                                                        VisionCommands
                                                                                                .blinkLimelights())))));
    }

    public static Command feedHome() {
        return IntakeCommands.intake()
                .withTimeout(0.15)
                .andThen(
                        Commands.either(
                                FeederCommands.addFeedRevolutions()
                                        .onlyIf(Robot.feeder.lasercan::intakedNote),
                                Commands.run(
                                        () ->
                                                RobotTelemetry.print(
                                                        "No lasercan found; Didn't feed")),
                                Robot.feeder.lasercan::validDistance));
    }

    public static Command laserFeedHome() {
        return FeederCommands.slowFeed()
                .until(() -> !Robot.feeder.intakedNote())
                .andThen(
                        FeederCommands.slowEject().until(() -> Robot.feeder.intakedNote()),
                        FeederCommands.stopMotor());
    }

    public static Command onDemandLaunching() {
        return LauncherCommands.runOnDemandVelocity()
                .alongWith(PivotCommands.onDemandPivot())
                .withName("RobotCommands.onDemandLaunching");
    }

    public static Command amp() {
        return FeederCommands.score()
                .withTimeout(0.1)
                .onlyIf(Robot.feeder::noteIsClose)
                .andThen(FeederCommands.feedToAmp())
                .alongWith(AmpTrapCommands.amp())
                .until(() -> Robot.ampTrap.hasNote())
                .andThen(
                        FeederCommands.stopMotor()
                                .alongWith(
                                        ElevatorCommands.amp(),
                                        AmpTrapCommands.amp()
                                                .withTimeout(0.65)
                                                .andThen(AmpTrapCommands.stopMotor())))
                .withName("RobotCommands.amp");
    }

    public static Command manualAmp() {
        return FeederCommands.feedToAmp().alongWith(AmpTrapCommands.amp());
    }

    public static Command eject() {
        return FeederCommands.eject()
                .alongWith(
                        AmpTrapCommands.eject(), IntakeCommands.eject(), LauncherCommands.eject())
                .withName("RobotCommands.eject");
    }

    public static Command score() {
        return Commands.either(
                        launchEject(),
                        hackDump(),
                        () -> Robot.leftLauncher.getMotorVelocityInRPM() > 10)
                .withName("RobotCommands.score");
    }

    public static Command launchEject() {
        return FeederCommands.launchEject()
                .alongWith(
                        AmpTrapCommands.score(),
                        IntakeCommands.slowIntake(),
                        ElevatorCommands.holdPosition())
                .withName("RobotCommands.launchEject");
    }

    // hack to avoid requiring dump subystems when we aren't running it in the score sequence
    public static Command hackDump() {
        return Commands.startEnd(
                        () -> {
                            if (Robot.leftLauncher.getMotorVelocityInRPM() < 10) {
                                dump().schedule();
                            }
                        },
                        () -> {
                            dump().cancel();
                        })
                .withName("RobotCommands.hackDump");
    }

    public static Command dump() {
        return FeederCommands.launchEject()
                .alongWith(
                        AmpTrapCommands.score(),
                        LauncherCommands.dump(),
                        IntakeCommands.slowIntake(),
                        ElevatorCommands.holdPosition())
                .withName("RobotCommands.dump");
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
                        Commands.startEnd(LEDs::turnOnCoastLEDs, LEDs::turnOffCoastLEDs)
                                .ignoringDisable(true))
                .withName("RobotCommands.coastModeMechanisms");
    }

    public static Command ensureBrakeMode() {
        return AmpTrapCommands.ensureBrakeMode()
                .alongWith(
                        ClimberCommands.ensureBrakeMode(),
                        ElevatorCommands.ensureBrakeMode(),
                        FeederCommands.ensureBrakeMode(),
                        IntakeCommands.ensureBrakeMode(),
                        LauncherCommands.ensureBrakeMode(),
                        PivotCommands.ensureBrakeMode())
                .withName("RobotCommands.ensureBrakeMode");
    }

    public static Command subwooferShot() {
        return LauncherCommands.subwoofer()
                .alongWith(PivotCommands.subwoofer())
                .withName("RobotCommands.subwooferReady");
    }

    public static Command podiumShot() {
        return LauncherCommands.deepShot()
                .alongWith(PivotCommands.podium(), PilotCommands.podiumAimingDrive())
                .withName("RobotCommands.podium");
    }

    public static Command ampWingShot() {
        return LauncherCommands.deepShot()
                .alongWith(PivotCommands.ampWing(), PilotCommands.ampWingAimingDrive())
                .withName("RobotCommands.ampWing");
    }

    public static Command intoAmpShot() {
        return LauncherCommands.intoAmp()
                .alongWith(PivotCommands.intoAmp())
                .withName("RobotCommands.intoAmp");
    }

    public static Command fromAmpShot() {
        return LauncherCommands.deepShot()
                .alongWith(PivotCommands.fromAmp(), PilotCommands.fromAmpAimingDrive())
                .withName("RobotCommands.ampWing");
    }

    public static Command topClimb() {
        return ClimberCommands.topClimb().alongWith(PivotCommands.home());
    }

    public static Command trapExtend() {
        return Commands.either(
                FeederCommands.score()
                        .withTimeout(0.1)
                        .onlyIf(Robot.feeder::noteIsClose)
                        .andThen(FeederCommands.feedToAmp())
                        .alongWith(AmpTrapCommands.amp())
                        .until(() -> Robot.ampTrap.hasNote())
                        .andThen(
                                AmpTrapCommands.stopMotor()
                                        .alongWith(
                                                FeederCommands.stopMotor(),
                                                ElevatorCommands.fullExtend())),
                ElevatorCommands.fullExtend(),
                Robot.ampTrap.lasercan::validDistance);
    }
}
