package frc.robot.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class AutoPaths {
    public static boolean front6Point5Note2 = false;
    public static boolean front6Point5Note2Picked = false;
    public static boolean source4Note2 = false;
    public static boolean Source4Note2Picked = false;
    public static boolean madtownPathRan = false;

    public static Command Front6Point5() {
        return AutoBuilder.buildAuto("Front 6.5")
                .until(() -> AutonCommands.isNoteIntaked())
                .andThen(
                        (AutonCommands.pathfindingCommandToPose(0, 0, 0, 0, 0)
                                        .until(() -> front6Point5Note2)
                                        .andThen(
                                                new InstantCommand(
                                                        () -> {
                                                            front6Point5Note2Picked = true;
                                                        })))
                                .andThen(AutoBuilder.buildAuto("Front 6.5 Part 2"))
                                .until(() -> AutonCommands.isNoteIntaked())
                                .andThen(AutonCommands.pathfindingCommandToPose(0, 0, 0, 0, 0))
                                .andThen(AutoBuilder.buildAuto("Front 6.5 Part 3"))
                                .onlyIf(() -> AutonCommands.isNoteIntaked()));
    }

    public static Command Source4() {
        return AutoBuilder.buildAuto("Source 4")
                .until(() -> !AutonCommands.isNoteIntaked())
                .andThen(
                        (AutonCommands.pathfindingCommandToPose(0, 0, 0, 0, 0)
                                        .until(() -> source4Note2)
                                        .andThen(
                                                new InstantCommand(
                                                        () -> {
                                                            Source4Note2Picked = true;
                                                        })))
                                .andThen(AutoBuilder.buildAuto("Source 4 Part 2"))
                                .until(() -> !AutonCommands.isNoteIntaked()))
                .andThen(AutonCommands.pathfindingCommandToPose(0, 0, 0, 0, 0))
                .andThen(AutoBuilder.buildAuto("Source 4 Part 3"));
    }

    public static Command FrontAlt6() {
        return AutoBuilder.buildAuto("Front Alt 6")
                .until(() -> AutonCommands.isNoteIntaked())
                .andThen(
                        (AutonCommands.pathfindingCommandToPose(7.05, 5.75, 130, 4, 2)
                                        .andThen(AutoBuilder.buildAuto("Front Alt 6 Part 2")))
                                .onlyIf(() -> AutonCommands.isNoteIntaked()));
    }

    public static Command Test() {
        return AutoBuilder.buildAuto("Detection Start")
                .andThen(
                        (AutonCommands.pathfindingCommandToPose(4.5, 6, 180, 2, 2))
                                .andThen(AutoBuilder.buildAuto("Detection Accept"))
                                .onlyIf(() -> Robot.feeder.intakedNote()))
                .andThen(
                        AutonCommands.pathfindingCommandToPose(4.5, 6, 180, 2, 2)
                                .andThen(AutoBuilder.buildAuto("Detection Decline"))
                                .onlyIf(() -> !Robot.feeder.intakedNote()));
    }

    public static Command MadtownTest() {
        return AutoBuilder.buildAuto("Madtown Detection Test")
                .onlyIf(() -> AutonCommands.noteInView())
                .andThen(
                        AutoBuilder.buildAuto("Madtown Detection Test 2")
                                .onlyIf(() -> !madtownPathRan));
    }
}
