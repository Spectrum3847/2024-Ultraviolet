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
        return new OnDemandPivot(pivot.config.test).withName("Pivot.onDemandPivot");
    }

    public static Command home() {
        return pivot.runPosition(pivot.config.home).withName("Pivot.home");
    }

    public static Command percentage() {
        return pivot.runManualOutput(0.1);
    }

    public static Command negativePercentage() {
        return pivot.runManualOutput(-0.1);
    }

    /* Intaking Positions */

    /* Scoring Positions */

    public static Command score() {
        return pivot.runPosition(pivot.config.score).withName("Pivot.score");
    }

    public static Command halfScore() {
        return pivot.runPosition(pivot.config.halfScore).withName("Pivot.halfScore");
    }

    public static Command subwoofer() {
        return pivot.runPosition(pivot.config.subwoofer).withName("Pivot.subwoofer");
    }

    public static Command ampScore() {
        return pivot.runPosition(pivot.config.amp).withName("Pivot.amp");
    }

    public static Command podium() {
        return pivot.runPosition(pivot.config.podium).withName("Pivot.podium");
    }

    public static Command autoLaunchPreload() {
        return pivot.runPosition(pivot.config.autoLaunchPreload).withName("Pivot.subwoofer");
    }

    public static Command autoLaunch2() {
        return pivot.runPosition(pivot.config.autoLaunch2).withName("Pivot.subwoofer");
    }

    public static Command autoLaunch3() {
        return pivot.runPosition(pivot.config.autoLaunch3).withName("Pivot.subwoofer");
    }

    public static Command stopMotor() {
        return pivot.runStop().withName("Pivot.stopMotor");
    }

    public static Command coastMode() {
        return pivot.coastMode();
    }
}
