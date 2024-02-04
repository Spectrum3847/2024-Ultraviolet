package frc.robot.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutonCommands {
    public static Command followSinglePath(String PathName) {
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile(PathName);

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    }

    public static Command trackNote() {
        return new InstantCommand(() -> {
            Auton.trackSpeaker = false;
            Auton.trackNote = true;
            System.out.println("Starting Tracking Note");
        }).withName("AutonCommands.trackNote");
    }

    public static Command trackSpeaker() {
        return new InstantCommand(() -> {
            Auton.trackNote = false;
            Auton.trackSpeaker = true;
            System.out.println("Starting Tracking Speaker");
        }).withName("AutonCommands.trackSpeaker");
    }

    public static Command stopTracking() {
        return new InstantCommand(() -> {
            Auton.trackNote = false;
            Auton.trackSpeaker = false;
            System.out.println("Stopping Tracking");
        }).withName("AutonCommands.stopTracking");
    }
}