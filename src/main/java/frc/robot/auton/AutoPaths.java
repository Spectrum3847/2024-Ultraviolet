package frc.robot.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoPaths {
    public static Command Front6Point5() {
        return AutoBuilder.buildAuto("Front 6.5").until(() -> !AutonCommands.isNoteIntaked()).andThen();
    }
}
