package frc.robot.mechanisms.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class PivotCommands {
    private static Pivot pivot = Robot.pivot;

    public static void setupDefaultCommand() {
        pivot.setDefaultCommand(pivot.runHoldPivot().withName("Pivot.default"));
    }

    /* Misc Positions */

    public static Command onDemandPivot() {
        return new OnDemandPivot(pivot.config.score).withName("Pivot.onDemandPivot");
    }

    public static Command home() {
        return pivot.runPosition(pivot.config.home).withName("Pivot.home");
    }

    /* Scoring Positions */

    public static Command subwoofer() {
        return pivot.runPosition(pivot.config.subwoofer).withName("Pivot.subwoofer");
    }

    public static Command podium() {
        return pivot.runPosition(pivot.config.podium).withName("Pivot.podium");
    }

    public static Command ampWing() {
        return pivot.runPosition(pivot.config.ampWing).withName("Pivot.ampWing");
    }

    /* Auto Positions */

    public static Command autoLaunchPreload() {
        return pivot.runPosition(pivot.config.autoLaunchPreload)
                .withName("Pivot.autoLaunchPreload");
    }

    public static Command autoLaunch2() {
        return pivot.runPosition(pivot.config.autoLaunch2).withName("Pivot.autoLaunch2");
    }

    public static Command autoLaunch3() {
        return pivot.runPosition(pivot.config.autoLaunch3).withName("Pivot.autoLaunch3");
    }

    /* Misc */

    public static Command stopMotor() {
        return pivot.runStop().withName("Pivot.stopMotor");
    }

    public static Command coastMode() {
        return pivot.coastMode();
    }
}
