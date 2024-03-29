package frc.robot.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoPaths {
    public static boolean front6Point5Note2 = false;
    public static boolean front6Point5Note2Picked = false;
    public static boolean source4Note2 = false;
    public static boolean Source4Note2Picked = false;

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

    public static Command FrontAlt6() {
        return AutoBuilder.buildAuto("Front Alt 6")
                .until(() -> !AutonCommands.isNoteIntaked())
                .andThen(
                        AutonCommands.pathfindingCommandToPose(0, 0, 0, 0, 0)
                                .andThen(AutoBuilder.buildAuto("Front Alt 6 Part 2"))
                                .until(() -> !AutonCommands.isNoteIntaked()));
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

    public static Command Test() {
        return AutoBuilder.buildAuto("New Auto")
                .until(() -> AutonCommands.isNoteIntaked())
                .andThen(
                        (AutonCommands.pathfindingCommandToPose(1, 1, 0, 2, 2)
                                        .andThen(AutoBuilder.buildAuto("New New Auto")))
                                .onlyIf(() -> AutonCommands.isNoteIntaked()));
    }
}
