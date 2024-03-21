package frc.robot.mechanisms.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;
import frc.robot.leds.LEDs;

public class PivotCommands {
    private static Pivot pivot = Robot.pivot;

    public static void setupDefaultCommand() {
        pivot.setDefaultCommand(pivot.runHoldPivot().withName("Pivot.default"));
    }

    /* Misc Positions */

    public static Command setPivotOnDistance(DoubleSupplier distance) {
        return pivot.runPosition(pivot.getAngleFromDistance(distance));
    }

    public static Command onDemandPivot() {
        return new OnDemandPivot(pivot.config.test).withName("Pivot.onDemandPivot");
    }

    public static Command home() {
        return pivot.runPosition(pivot.config.home).alongWith(sendPivotFeedback()).withName("Pivot.home");
    }

    public static Command climb() {
        return pivot.runPosition(pivot.config.climb).alongWith(sendPivotFeedback()).withName("Pivot.climb");
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
        return pivot.runPosition(pivot.config.amp).alongWith(sendPivotFeedback()).withName("Pivot.amp");
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

    public static Command autoLaunchSub() {
        return pivot.runPosition(pivot.config.subwoofer).withName("Pivot.subwoofer");
    }

    public static Command stopMotor() {
        return pivot.runStop().withName("Pivot.stopMotor");
    }

    public static Command coastMode() {
        return pivot.coastMode();
    }

    // /*Helper Command */
    public static Command sendPivotFeedback() {
        return new FunctionalCommand(
                () -> {},
                () -> {
                    if (pivot.getMotorPercentAngle() < 1) {
                        Commands.startEnd(LEDs::turnOnHomeLEDs, LEDs::turnOffHomeLEDs)
                                .withTimeout(1.5);
                    }

                    else if ((pivot.getMotorPercentAngle() < pivot.config.amp +1)&& ( pivot.getMotorPercentAngle() > pivot.config.amp -1)) {
                        Commands.startEnd(LEDs::turnOnAmpLEDs, LEDs::turnOffAmpLEDs)
                                .withTimeout(1.5);
                    }

                    else if ((pivot.getMotorPercentAngle() < pivot.config.climb +1 ) && (pivot.getMotorPercentAngle() > pivot.config.amp -1)) {
                        Commands.startEnd(LEDs::turnOnClimbLEDs, LEDs::turnOffClimbLEDs)
                                .withTimeout(1.5);
                    }
                
                },
                (b) -> {},
                () -> false);
    } 
}
