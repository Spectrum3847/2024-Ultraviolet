package frc.robot.mechanisms.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class PivotCommands {
    private static Pivot pivot = Robot.pivot;

    public static void setupDefaultCommand() {
        pivot.setDefaultCommand(pivot.runHoldPivot().withName("Pivot.default"));
    }

    /* Misc Positions */

    public static Command home() {
        return pivot.runPosition(pivot.config.home).withName("Pivot.home");
    }

    /* Intaking Positions */

    /* Scoring Positions */

    public static Command score() {
        return pivot.runPosition(pivot.config.score).withName("Pivot.score");
    }

    public static Command halfScore() {
        return pivot.runPosition(pivot.config.halfScore).withName("Pivot.halfScore");
    }
}
