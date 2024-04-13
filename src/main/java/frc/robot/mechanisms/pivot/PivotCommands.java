package frc.robot.mechanisms.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;

public class PivotCommands {
    private static Pivot pivot = Robot.pivot;

    public static void setupDefaultCommand() {
        pivot.setDefaultCommand(
                pivot.runHoldPivot().ignoringDisable(true).withName("Pivot.default"));
    }

    /* Misc Positions */

    public static Command setPivotOnDistance(DoubleSupplier distance) {
        return pivot.runPosition(pivot.getAngleFromDistance(distance));
    }

    public static Command setPivotOnFeedDistance(DoubleSupplier distance) {
        return pivot.runPosition(pivot.getAngleFromFeedDistance(distance));
    }

    public static Command setPivotOnDeepFeedDistance(DoubleSupplier distance) {
        return pivot.runPosition(pivot.getAngleFromDeepFeedDistance(distance));
    }

    public static Command onDemandPivot() {
        return new OnDemandPivot(pivot.config.score).withName("Pivot.onDemandPivot");
    }

    public static Command home() {
        return pivot.runPosition(pivot.config.home).withName("Pivot.home");
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

    public static Command podium() {
        return pivot.runPosition(pivot.config.podium).withName("Pivot.podium");
    }

    public static Command ampWing() {
        return pivot.runPosition(pivot.config.ampWing).withName("Pivot.ampWing");
    }

    public static Command fromAmp() {
        return pivot.runPosition(pivot.config.fromAmp).withName("Pivot.fromAmp");
    }

    public static Command intoAmp() {
        return pivot.runPosition(pivot.config.intoAmp).withName("Pivot.intoAmp");
    }

    public static Command autoLaunchPreload() {
        return pivot.runPosition(pivot.config.autoLaunchPreload).withName("Pivot.subwoofer");
    }

    public static Command autoLaunch1() {
        return pivot.runPosition(pivot.config.autoLaunch1).withName("Pivot.subwoofer");
    }

    public static Command autoLaunch2() {
        return pivot.runPosition(pivot.config.autoLaunch2).withName("Pivot.subwoofer");
    }

    public static Command autoLaunch3() {
        return pivot.runPosition(pivot.config.autoLaunch3).withName("Pivot.subwoofer");
    }

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

    public static Command spitReady() {
        return pivot.runPosition(pivot.config.spitReady).withName("Pivot.subwoofer");
    }

    public static Command intake() {
        return pivot.runPosition(pivot.config.intake).withName("Pivot.intake");
    }

    public static Command coastMode() {
        return pivot.coastMode();
    }

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
    }
}
