package frc.robot.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotTelemetry;
import frc.robot.mechanisms.amptrap.AmpTrapCommands;
import frc.robot.mechanisms.feeder.FeederCommands;
import frc.robot.mechanisms.intake.IntakeCommands;
import frc.robot.mechanisms.launcher.LauncherCommands;
import frc.robot.mechanisms.pivot.PivotCommands;

public class AutonCommands {
    public static Command followSinglePath(String PathName) {
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile(PathName);

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    }

    public static Command trackNote() {
        return new InstantCommand(
                        () -> {
                            Auton.trackSpeaker = false;
                            Auton.trackNote = true;
                            System.out.println("Starting Tracking Note");
                        })
                .withName("AutonCommands.trackNote");
    }

    public static Command trackSpeaker() {
        return new InstantCommand(
                        () -> {
                            Auton.trackNote = false;
                            Auton.trackSpeaker = true;
                            System.out.println("Starting Tracking Speaker");
                        })
                .withName("AutonCommands.trackSpeaker");
    }

    public static Command stopTracking() {
        return new InstantCommand(
                        () -> {
                            Auton.trackNote = false;
                            Auton.trackSpeaker = false;
                            System.out.println("Stopping Tracking");
                        })
                .withName("AutonCommands.stopTracking");
    }

    public static Command stopFeed() {
        return IntakeCommands.stopMotor()
                .alongWith(AmpTrapCommands.stopMotor(), FeederCommands.stopMotor())
                .withName("AutonCommands.stopFeed");
    }

    public static Command launchReadyPreload() {
        return PivotCommands.autoLaunchPreload()
                .alongWith(LauncherCommands.subwoofer())
                .withName("AutonCommands.launchReadyPreload");
    }

    public static Command launchReady1() {
        return PivotCommands.autoLaunch1()
                .alongWith(LauncherCommands.subwoofer())
                .withName("AutonCommands.launchReadyPreload");
    }

    public static Command launchReady2() {
        return PivotCommands.autoLaunch2()
                .alongWith(LauncherCommands.subwoofer())
                .withName("AutonCommands.launchReady2");
    }

    public static Command launchReady3() {
        return PivotCommands.autoLaunch3()
                .alongWith(LauncherCommands.subwoofer())
                .withName("AutonCommands.launchReady3");
    }

    public static Command launchReadySubwoofer() {
        return (PivotCommands.subwoofer().alongWith(LauncherCommands.subwoofer()))
                .withTimeout(1)
                .withName("AutonCommands.launchReady3");
    }

    public static Command launch() {
        return FeederCommands.autoFeed()
                .withTimeout(0.35)
                .andThen(FeederCommands.stopMotor().withTimeout(0.01))
                .withName("AutonCommands.launch");
    }

    public static Command intake() {
        return IntakeCommands.intake()
                .alongWith(AmpTrapCommands.intake())
                .until(Robot.feeder::intakedNote)
                .deadlineWith(
                        PivotCommands.intake()
                                .onlyIf(
                                        () ->
                                                Robot.pivot.getMotorPercentAngle()
                                                        < Robot.pivot.config.intake))
                .andThen(Commands.waitSeconds(0.3), IntakeCommands.stopMotor())
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

    public static Command visionLaunch() {
        return LauncherCommands.distanceVelocity(() -> Robot.vision.getSpeakerDistance())
                .alongWith(
                        PivotCommands.setPivotOnDistance(() -> Robot.vision.getSpeakerDistance()));
    }
}
