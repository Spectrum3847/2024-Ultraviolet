package frc.robot.mechanisms.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Robot;
<<<<<<< HEAD
import frc.robot.leds.LEDs;
=======
>>>>>>> Madtown-Auto
import java.util.function.DoubleSupplier;

public class PivotCommands {
    private static Pivot pivot = Robot.pivot;

    public static void setupDefaultCommand() {
        pivot.setDefaultCommand(
<<<<<<< HEAD
                pivot.runHoldPivot().withName("Pivot.default").ignoringDisable(true));
=======
                pivot.runHoldPivot().ignoringDisable(true).withName("Pivot.default"));
>>>>>>> Madtown-Auto
    }

    /* Misc Positions */

    public static Command setPivotOnDistance(DoubleSupplier distance) {
        return pivot.runPosition(pivot.getAngleFromDistance(distance));
    }

<<<<<<< HEAD
=======
    public static Command setPivotOnFeedDistance(DoubleSupplier distance) {
        return pivot.runPosition(pivot.getAngleFromFeedDistance(distance));
    }

    public static Command setPivotOnDeepFeedDistance(DoubleSupplier distance) {
        return pivot.runPosition(pivot.getAngleFromDeepFeedDistance(distance));
    }

>>>>>>> Madtown-Auto
    public static Command onDemandPivot() {
        return new OnDemandPivot(pivot.config.score).withName("Pivot.onDemandPivot");
    }

    public static Command home() {
        return pivot.runPosition(pivot.config.home)
                .alongWith(sendPivotFeedback())
                .withName("Pivot.home");
    }

    public static Command climb() {
        return pivot.runPosition(pivot.config.climb)
                .alongWith(sendPivotFeedback())
                .withName("Pivot.climb");
    }

    public static Command climbHome() {
        return pivot.runPosition(pivot.config.climbHome).withName("Pivot.climbHome");
    }

    public static Command manualFeed() {
        return pivot.runPosition(pivot.config.manualFeed).withName("Pivot.manualFeed");
    }

    /* Scoring Positions */

    public static Command subwoofer() {
        return pivot.runPosition(pivot.config.subwoofer).withName("Pivot.subwoofer");
    }

    public static Command ampScore() {
        return pivot.runPosition(pivot.config.amp)
                .alongWith(sendPivotFeedback())
                .withName("Pivot.amp");
    }

    public static Command podium() {
        return pivot.runPosition(pivot.config.podium).withName("Pivot.podium");
    }

<<<<<<< HEAD
=======
    public static Command ampWing() {
        return pivot.runPosition(pivot.config.ampWing).withName("Pivot.ampWing");
    }

    public static Command fromAmp() {
        return pivot.runPosition(pivot.config.fromAmp).withName("Pivot.fromAmp");
    }

    public static Command intoAmp() {
        return pivot.runPosition(pivot.config.intoAmp).withName("Pivot.intoAmp");
    }

>>>>>>> Madtown-Auto
    public static Command autoLaunchPreload() {
        return pivot.runPosition(pivot.config.autoLaunchPreload).withName("Pivot.subwoofer");
    }

<<<<<<< HEAD
=======
    public static Command autoLaunchPreload2() {
        return pivot.runPosition(pivot.config.autoLaunchPreload2).withName("Pivot.subwoofer");
    }

    public static Command autoLaunch1() {
        return pivot.runPosition(pivot.config.autoLaunch1).withName("Pivot.subwoofer");
    }

>>>>>>> Madtown-Auto
    public static Command autoLaunch2() {
        return pivot.runPosition(pivot.config.autoLaunch2).withName("Pivot.subwoofer");
    }

    public static Command autoLaunch3() {
        return pivot.runPosition(pivot.config.autoLaunch3).withName("Pivot.subwoofer");
    }

<<<<<<< HEAD
    public static Command autoLaunchSub() {
        return pivot.runPosition(pivot.config.subwoofer).withName("Pivot.subwoofer");
    }

    public static Command stopMotor() {
        return pivot.runStop().withName("Pivot.stopMotor");
=======
    public static Command autoLaunch4() {
        return pivot.runPosition(pivot.config.autoLaunch4).withName("Pivot.subwoofer");
    }

    public static Command autoLaunch5() {
        return pivot.runPosition(pivot.config.autoLaunch5).withName("Pivot.subwoofer");
    }

    public static Command autoLaunch6() {
        return pivot.runPosition(pivot.config.autoLaunch6).withName("Pivot.subwoofer");
    }

    public static Command autoLaunch7() {
        return pivot.runPosition(pivot.config.autoLaunch7).withName("Pivot.subwoofer");
    }

    public static Command autoLaunch8() {
        return pivot.runPosition(pivot.config.autoLaunch8).withName("Pivot.subwoofer");
    }

    public static Command autoLaunch9() {
        return pivot.runPosition(pivot.config.autoLaunch9).withName("Pivot.subwoofer");
    }

    public static Command autoLaunch10() {
        return pivot.runPosition(pivot.config.autoLaunch10).withName("Pivot.subwoofer");
    }

    public static Command autoLaunch11() {
        return pivot.runPosition(pivot.config.autoLaunch11).withName("Pivot.subwoofer");
    }

    public static Command autoLaunch12() {
        return pivot.runPosition(pivot.config.autoLaunch12).withName("Pivot.subwoofer");
    }

    public static Command autoLaunch13() {
        return pivot.runPosition(pivot.config.autoLaunch13).withName("Pivot.subwoofer");
    }

    public static Command autoLaunch14() {
        return pivot.runPosition(pivot.config.autoLaunch14).withName("Pivot.subwoofer");
    }

    public static Command autoLaunch15() {
        return pivot.runPosition(pivot.config.autoLaunch15).withName("Pivot.subwoofer");
    }

    public static Command autoLaunch16() {
        return pivot.runPosition(pivot.config.autoLaunch16).withName("Pivot.subwoofer");
    }

    public static Command autoLaunch17() {
        return pivot.runPosition(pivot.config.autoLaunch17).withName("Pivot.subwoofer");
    }

    public static Command spitReady() {
        return pivot.runPosition(pivot.config.spitReady).withName("Pivot.subwoofer");
    }

    public static Command spitReady2() {
        return pivot.runPosition(pivot.config.spitReady2).withName("Pivot.subwoofer");
    }

    public static Command intake() {
        return pivot.runPosition(pivot.config.intake).withName("Pivot.intake");
>>>>>>> Madtown-Auto
    }

    public static Command coastMode() {
        return pivot.coastMode();
    }

<<<<<<< HEAD
    // /*Helper Command */
    public static Command sendPivotFeedback() {
        return new FunctionalCommand(
                () -> {},
                () -> {
                    if (pivot.getMotorPercentAngle() < 1) {
                        Commands.startEnd(LEDs::turnOnHomeLEDs, LEDs::turnOffHomeLEDs)
                                .withTimeout(1.5);
                    } else if ((pivot.getMotorPercentAngle() < pivot.config.amp + 1)
                            && (pivot.getMotorPercentAngle() > pivot.config.amp - 1)) {
                        Commands.startEnd(LEDs::turnOnAmpLEDs, LEDs::turnOffAmpLEDs)
                                .withTimeout(1.5);
                    } else if ((pivot.getMotorPercentAngle() < pivot.config.climb + 1)
                            && (pivot.getMotorPercentAngle() > pivot.config.amp - 1)) {
                        Commands.startEnd(LEDs::turnOnClimbLEDs, LEDs::turnOffClimbLEDs)
                                .withTimeout(1.5);
                    }
                },
                (b) -> {},
                () -> false);
=======
    public static Command stopMotor() {
        return pivot.runStop().withName("Pivot.stop");
    }

    public static Command ensureBrakeMode() {
        return pivot.ensureBrakeMode();
    }

    /** increase vision shots by 0.5 percent */
    public static Command increaseOffset() {
        return pivot.runOnce(pivot::increaseOffset)
                .withName("Pivot.increaseFudgeFactor")
                .ignoringDisable(true);
    }

    /** decrease vision shots by 0.5 percent */
    public static Command decreaseOffset() {
        return pivot.runOnce(pivot::decreaseOffset)
                .withName("Pivot.decreaseFudgeFactor")
                .ignoringDisable(true);
    }

    /** reset fudge factor to 0 */
    public static Command resetOffset() {
        return pivot.runOnce(pivot::resetOffset)
                .withName("Pivot.resetFudgeFactor")
                .ignoringDisable(true);
    }

    public static Command switchFeedSpot() {
        return pivot.runOnce(pivot::switchFeedSpot)
                .withName("Pivot.switchFeedSpot")
                .ignoringDisable(true);
>>>>>>> Madtown-Auto
    }
}
