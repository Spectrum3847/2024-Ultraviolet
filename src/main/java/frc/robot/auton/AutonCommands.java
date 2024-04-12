package frc.robot.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
import frc.robot.vision.VisionCommands;

public class AutonCommands {
    public static Command followSinglePath(String PathName) {
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile(PathName);

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    }

    public static Command pathfindingCommandToPose(
            double xPos, double yPos, double rotation, double vel, double accel) {
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        Pose2d targetPose = new Pose2d(xPos, yPos, Rotation2d.fromDegrees(rotation));

        // Create the constraints to use while pathfinding
        PathConstraints constraints =
                new PathConstraints(
                        vel, accel, Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand =
                AutoBuilder.pathfindToPoseFlipped(
                        targetPose,
                        constraints,
                        0.0, // Goal end velocity in meters/sec
                        0.0 // Rotation delay distance in meters. This is how far the robot should
                        // travel before attempting to rotate.
                        );

        return pathfindingCommand;
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

    public static Command spit() {
        return LauncherCommands.autoDump()
                .alongWith(FeederCommands.autoFeed())
                .withName("AutoCommands.spit");
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

    public static Command launchReady4() {
        return PivotCommands.autoLaunch4()
                .alongWith(LauncherCommands.subwoofer())
                .withName("AutonCommands.launchReady4");
    }

    public static Command launchReady5() {
        return PivotCommands.autoLaunch5()
                .alongWith(LauncherCommands.subwoofer())
                .withName("AutonCommands.launchReady5");
    }

    public static Command launchReady6() {
        return PivotCommands.autoLaunch6()
                .alongWith(LauncherCommands.subwoofer())
                .withName("AutonCommands.launchReady6");
    }

    public static Command launchReady7() {
        return PivotCommands.autoLaunch7()
                .alongWith(LauncherCommands.subwoofer())
                .withName("AutonCommands.launchReady7");
    }

    public static Command launchReady8() {
        return PivotCommands.autoLaunch8()
                .alongWith(LauncherCommands.subwoofer())
                .withName("AutonCommands.launchReady7");
    }

    public static Command launchReady9() {
        return PivotCommands.autoLaunch9()
                .alongWith(LauncherCommands.subwoofer())
                .withName("AutonCommands.launchReady7");
    }

    public static Command launchReady10() {
        return PivotCommands.autoLaunch10()
                .alongWith(LauncherCommands.subwoofer())
                .withName("AutonCommands.launchReady7");
    }

    public static Command launchReady11() {
        return PivotCommands.autoLaunch11()
                .alongWith(LauncherCommands.subwoofer())
                .withName("AutonCommands.launchReady7");
    }

    public static Command launchReady12() {
        return PivotCommands.autoLaunch12()
                .alongWith(LauncherCommands.subwoofer())
                .withName("AutonCommands.launchReady7");
    }

    public static Command launchReadySubwoofer() {
        return (PivotCommands.subwoofer().alongWith(LauncherCommands.subwoofer()))
                .withTimeout(0.5)
                .withName("AutonCommands.launchReady3");
    }

    public static Command launch() {
        return FeederCommands.autoFeed()
                .withTimeout(0.35)
                .andThen(FeederCommands.stopMotor().withTimeout(0.01))
                .withName("AutonCommands.launch");
    }

    public static Command launchShort() {
        return FeederCommands.autoFeed()
                .withTimeout(0.1)
                .andThen(FeederCommands.stopMotor().withTimeout(0.01))
                .withName("AutonCommands.launch");
    }

    public static Command spitReady() {
        return PivotCommands.spitReady().withName("AutonCommands.spitReady");
    }

    public static Command intake() {
        return new InstantCommand(
                        () -> {
                            Auton.noteIntaked = false;
                        })
                .andThen(
                        IntakeCommands.intake()
                                .alongWith(AmpTrapCommands.intake())
                                .until(Robot.feeder::intakedNote)
                                .andThen(
                                        new InstantCommand(
                                                        () -> {
                                                            Auton.noteIntaked = true;
                                                        })
                                                .alongWith(
                                                        IntakeCommands.intake()
                                                                .withTimeout(0.15)
                                                                .andThen(
                                                                        Commands.either(
                                                                                FeederCommands
                                                                                        .addFeedRevolutions()
                                                                                        .onlyIf(
                                                                                                Robot
                                                                                                                .feeder
                                                                                                                .lasercan
                                                                                                        ::intakedNote),
                                                                                Commands.run(
                                                                                        () ->
                                                                                                RobotTelemetry
                                                                                                        .print(
                                                                                                                "No lasercan found; Didn't feed")),
                                                                                Robot.feeder
                                                                                                .lasercan
                                                                                        ::validDistance)))));
    }

    public static Command visionLaunch() {
        return LauncherCommands.distanceVelocity(() -> Robot.vision.getSpeakerDistance())
                .alongWith(
                        PivotCommands.setPivotOnDistance(() -> Robot.vision.getSpeakerDistance()));
    }

    public static boolean isNoteIntaked() {
        if (Auton.intakeCheck == true) {
            if (Auton.noteIntaked == true) {
                return false;
            } else if (Auton.noteIntaked == false) {
                return true;
            }
        } else {
            return false;
        }
        return false;
    }

    public static Command intakeCheck() {
        return new InstantCommand(
                        () -> {
                            Auton.intakeCheck = true;
                            System.out.println("Starting Intake Check");
                        })
                .withName("AutonCommands.intakeCheck");
    }

    public static Command stopIntakeCheck() {
        return new InstantCommand(
                        () -> {
                            Auton.intakeCheck = false;
                            System.out.println("Starting Stop Intake Check");
                        })
                .withName("AutonCommands.stopIntakeCheck");
    }

    public static Command intakeFeed() {
        return IntakeCommands.runFull()
                .alongWith(AmpTrapCommands.intake().alongWith(FeederCommands.autoFeed()));
    }

    public static Command resetPoseToVision() {
        return VisionCommands.resetPoseToVision().withTimeout(0.01);
    }
}
