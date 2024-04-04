package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.crescendo.Field;
import frc.robot.leds.LEDs;
import frc.robot.leds.LEDsCommands;
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

    public static void setupRobotTriggers() {
        Trigger coastMode =
                new Trigger(
                        () ->
                                DriverStation.isDisabled()
                                        && Robot.feeder.lasercan.intakedNote()
                                        && (Robot.ampTrap.bottomLasercan.closeNote()
                                                || Robot.ampTrap.topLasercan.closeNote()));
        coastMode.toggleOnTrue(RobotCommands.coastModeMechanisms());
    }

    // score speaker if in range, otherwise launch to feed
    public static Command visionLaunch() {
        return Commands.either(
                        visionSpeakerLaunch(),
                        visionFeedLaunch(),
                        () -> {
                            if (Field.isBlue()) {
                                return Robot.swerve.getPose().getTranslation().getX()
                                        <= (Field.fieldLength / 2) - 1;
                            } else {
                                return Robot.swerve.getPose().getTranslation().getX()
                                        >= (Field.fieldLength / 2) + 1;
                            }
                        })
                .withName("RobotCommands.visionLaunch");
    }

    // Auto aim, set pivot, and launcher velocities
    public static Command visionSpeakerLaunch() {
        return PilotCommands.aimToSpeaker()
                .alongWith(
                        LauncherCommands.distanceVelocity(() -> Robot.vision.getSpeakerDistance()),
                        PivotCommands.setPivotOnDistance(() -> Robot.vision.getSpeakerDistance()),
                        Commands.startEnd(Robot.vision::setAiming, Robot.vision::setNotAiming))
                .withName("RobotCommands.visionLaunch");
    }

    // Auto aim, set pivot, and launcher velocities
    public static Command visionFeedLaunch() {
        return PilotCommands.aimToFeed()
                .alongWith(
                        LauncherCommands.feedDistanceVelocity(() -> Robot.vision.getFeedDistance()),
                        PivotCommands.setPivotOnFeedDistance(() -> Robot.vision.getFeedDistance()))
                .withName("RobotCommands.visionLaunch");
    }

    public static Command testMap() {
        return PilotCommands.aimToFeed().alongWith(RobotCommands.onDemandLaunching());
    }

    public static Command feedLaunch() {
        return PilotCommands.aimToFeed()
                .alongWith(
                        LauncherCommands.feedDistanceVelocity(() -> Robot.vision.getFeedDistance()),
                        PivotCommands.setPivotOnFeedDistance(() -> Robot.vision.getFeedDistance()))
                .withName("RobotCommands.feedLaunch");
    }

    // Run the intake for at least 0.4 seconds OR until the lasercan sees the note,
    // (also get pivot
    // to a good position if it's
    // too low, and stop it when intaking is done), then rumble and blink
    public static Command smartIntake() {
        return ElevatorCommands.home()
                .alongWith(
                        IntakeCommands.intake()
                                .alongWith(AmpTrapCommands.intake())
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
                                                                                        RobotCommands
                                                                                                .blinkGreen())))));
    }

    public static Command autoFeed() {
        return RobotCommands.visionFeedLaunch()
                .alongWith(launchFromIntake())
                .withName("RobotCommands.autoFeed");
    }

    public static Command launchFromIntake() {
        return IntakeCommands.runFull()
                .alongWith(AmpTrapCommands.intake(), FeederCommands.ejectFromIntake())
                .withName("RobotCommands.feedShoot");
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

    // Amp motor doesn't run if already have elevator up
    public static Command amp() {
        return FeederCommands.score()
                .withTimeout(0.1)
                .onlyIf(Robot.feeder::noteIsClose)
                .andThen(FeederCommands.feedToAmp())
                .alongWith(AmpTrapCommands.amp())
                .until(
                        () ->
                                // Robot.ampTrap.getBotLaserCanDistance() >= 20
                                Robot.ampTrap.getTopLaserCanDistance() <= 10)
                .andThen(FeederCommands.stopMotor().alongWith(AmpTrapCommands.stopMotor()))
                .alongWith(
                        ElevatorCommands.amp()
                                .onlyIf(() -> Robot.ampTrap.getBotLaserCanDistance() < 200)
                                .repeatedly())
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
                        PivotCommands.coastMode(),
                        ClimberCommands.coastMode(),
                        ElevatorCommands.coastMode(),
                        FeederCommands.coastMode(),
                        IntakeCommands.coastMode(),
                        LauncherCommands.coastMode(),
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
                        PivotCommands.ensureBrakeMode(),
                        Commands.runOnce(LEDs::turnOffCoastLEDs).ignoringDisable(true))
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
        return Commands.either(
                ClimberCommands.topClimb()
                        .alongWith(
                                PivotCommands.climbHome(),
                                FeederCommands.score()
                                        .withTimeout(0.1)
                                        .onlyIf(Robot.feeder::noteIsClose)
                                        .andThen(FeederCommands.feedToAmp())
                                        .alongWith(AmpTrapCommands.amp())
                                        .until(
                                                () ->
                                                        Robot.ampTrap.bottomHasNote()
                                                                || Robot.ampTrap.topHasNote())
                                        .onlyIf(() -> !Robot.ampTrap.bottomHasNote())
                                        .andThen(
                                                AmpTrapCommands.stopMotor()
                                                        .alongWith(FeederCommands.stopMotor()))),
                ClimberCommands.topClimb().alongWith(PivotCommands.climbHome()),
                Robot.ampTrap.bottomLasercan::validDistance);
    }

    public static Command trapExtend() {
        return Commands.either(
                FeederCommands.score()
                        .withTimeout(0.1)
                        .onlyIf(Robot.feeder::noteIsClose)
                        .andThen(FeederCommands.feedToAmp())
                        .alongWith(AmpTrapCommands.amp())
                        .until(() -> Robot.ampTrap.bottomHasNote() || Robot.ampTrap.topHasNote())
                        .onlyIf(() -> !Robot.ampTrap.bottomHasNote())
                        .andThen(
                                AmpTrapCommands.stopMotor()
                                        .alongWith(
                                                FeederCommands.stopMotor(),
                                                ElevatorCommands.fullExtend())),
                ElevatorCommands.fullExtend(),
                Robot.ampTrap.bottomLasercan::validDistance);
    }

    public static Command centerClimbAlign() {
        return ClimberCommands.topClimb()
                .alongWith(
                        PivotCommands.climbHome(),
                        FeederCommands.score()
                                .withTimeout(0.1)
                                .onlyIf(Robot.feeder::noteIsClose)
                                .andThen(FeederCommands.feedToAmp())
                                .alongWith(AmpTrapCommands.amp())
                                .until(
                                        () ->
                                                Robot.ampTrap.bottomHasNote()
                                                        || Robot.ampTrap.topHasNote())
                                .onlyIf(() -> !Robot.ampTrap.bottomHasNote())
                                .andThen(
                                        AmpTrapCommands.stopMotor()
                                                .alongWith(FeederCommands.stopMotor())));
    }

    public static Command autoClimb() {
        return ClimberCommands.midClimb()
                .onlyIf(() -> !Robot.elevator.isElevatorUp())
                .until(
                        () ->
                                Robot.climber.getMotorPercentAngle()
                                        < Robot.climber.config.midClimb + 4)
                .andThen(
                        SwerveCommands.Drive(
                                        () -> 0.1 * Robot.swerve.config.maxVelocity,
                                        () -> 0,
                                        () -> 0,
                                        () -> false, // true is field oriented
                                        () -> true)
                                .onlyIf(() -> !Robot.elevator.isElevatorUp())
                                .withTimeout(1)
                                .andThen(PilotCommands.pilotDrive())
                                .alongWith(
                                        Commands.waitSeconds(0.5)
                                                .andThen(
                                                        ElevatorCommands.fullExtend()
                                                                .withTimeout(0.5),
                                                        ClimberCommands.botClimb())))
                .until(
                        () ->
                                Robot.climber.getMotorPercentAngle()
                                        < Robot.climber.config.botClimb + 4)
                .andThen(AmpTrapCommands.amp());
    }

    public static Command manualSource() {
        return LauncherCommands.manualSource()
                .alongWith(FeederCommands.manualSource())
                .withName("RobotCommands.manualSource");
    }

    public static Command blinkGreen() {
        return LEDsCommands.intakeReadyStrobe()
                .alongWith(VisionCommands.blinkLimelights())
                .withName("RobotCommands.blinkGreen");
    }
}
