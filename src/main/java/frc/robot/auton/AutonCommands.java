package frc.robot.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
                .alongWith(AmpTrapCommands.stopMotor(), FeederCommands.stopMotor());
    }

    public static Command launchReady() {
        return LauncherCommands.subwoofer()
                .withTimeout(0.3)
                .withName("RobotCommands.launchReadyPreload");
    }

    public static Command pivotReadyPreload() {
        return PivotCommands.autoLaunchPreload().withName("RobotCommands.pivotReadyPreload");
    }

    public static Command pivotReady2() {
        return PivotCommands.autoLaunch2().withName("RobotCommands.pivotReady2");
    }

    public static Command pivotReady3() {
        return PivotCommands.autoLaunch3().withName("RobotCommands.pivotReady3");
    }

    public static Command pivotReadySub() {
        return PivotCommands.autoLaunchSub().withName("RobotCommands.pivotReadyySub");
    }

    public static Command score1() {
        return ((FeederCommands.feeder().alongWith(AutonCommands.pivotReadyPreload()))
                        .withTimeout(.05))
                .andThen(
                        (FeederCommands.stop()
                                .alongWith(AutonCommands.pivotReadyPreload())
                                .withTimeout(0.4)))
                .andThen(
                        (FeederCommands.feeder().alongWith(AutonCommands.pivotReadyPreload()))
                                .withTimeout(.3))
                .andThen(FeederCommands.stop().withTimeout(0.01));
    }

    public static Command score2() {
        return ((FeederCommands.feeder().alongWith(AutonCommands.pivotReady2())).withTimeout(.05))
                .andThen(
                        (FeederCommands.stop()
                                .alongWith(AutonCommands.pivotReady2())
                                .withTimeout(0.4)))
                .andThen(
                        (FeederCommands.feeder().alongWith(AutonCommands.pivotReady2()))
                                .withTimeout(.3))
                .andThen(FeederCommands.stop().withTimeout(0.01));
    }

    public static Command score3() {
        return ((FeederCommands.feeder().alongWith(AutonCommands.pivotReady3())).withTimeout(.05))
                .andThen(
                        (FeederCommands.stop()
                                .alongWith(AutonCommands.pivotReady3())
                                .withTimeout(0.4)))
                .andThen(
                        (FeederCommands.feeder().alongWith(AutonCommands.pivotReady3()))
                                .withTimeout(.3))
                .andThen(FeederCommands.stop().withTimeout(0.01));
    }

    public static Command scoreSub() {
        return ((FeederCommands.feeder().alongWith(AutonCommands.pivotReadySub())).withTimeout(.05))
                .andThen(
                        (FeederCommands.stop()
                                .alongWith(AutonCommands.pivotReadySub())
                                .withTimeout(0.4)))
                .andThen(
                        (FeederCommands.feeder().alongWith(AutonCommands.pivotReadySub()))
                                .withTimeout(.2))
                .andThen(FeederCommands.stop().withTimeout(0.01));
    }
}
