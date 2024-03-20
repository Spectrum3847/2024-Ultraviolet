package frc.robot.pilot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.crescendo.Field;
import frc.robot.Robot;
import frc.robot.leds.LEDs;
import frc.robot.mechanisms.climber.Climber;
import frc.robot.mechanisms.launcher.LauncherCommands;
import frc.robot.mechanisms.pivot.Pivot;
import frc.robot.swerve.Swerve;
import frc.robot.swerve.commands.SwerveCommands;

/** This class should have any command calls that directly call the Pilot */
public class PilotCommands {
    private static Pilot pilot = Robot.pilot;
    private static Climber climber = Robot.climber;
    private static Pivot pivot = Robot.pivot;
    private static Swerve swerve = Robot.swerve;

    /** Set default command to turn off the rumble */
    public static void setupDefaultCommand() {
        Robot.pilot.setDefaultCommand(launchReadyRumble().withName("Pilot.default"));
    }

    /** Command that can be used to rumble the pilot controller */
    public static Command rumble(double intensity, double durationSeconds) {
        return Robot.pilot.rumbleCommand(intensity, durationSeconds);
    }

    public static Command launchReadyRumble() {
        return new FunctionalCommand(
                        () -> {},
                        () -> {
                            if (LauncherCommands.isAtSpeed) {
                                if (swerve.rotationControllerAtFeedback()) {
                                    Robot.pilot.controller.rumbleController(1, 1);
                                } else {
                                    Robot.pilot.controller.rumbleController(0, 0);
                                }
                                LEDs.turnOnLaunchLEDs();
                            } else {
                                Robot.pilot.controller.rumbleController(0, 0);
                                LEDs.turnOffLaunchLEDs();
                            }
                        },
                        (b) -> {},
                        () -> false,
                        Robot.pilot)
                .ignoringDisable(true);
    }

    /** Full control of the swerve by the Pilot command */
    public static Command pilotDrive() {
        return SwerveCommands.Drive(
                        () -> pilot.getDriveFwdPositive(),
                        () -> pilot.getDriveLeftPositive(),
                        () -> -pilot.getDriveCCWPositive(),
                        () -> pilot.getFieldOriented(), // true is field oriented
                        () -> true)
                .withName("Swerve.PilotDrive");
    }

    public static Command fullTurnDrive() {
        return SwerveCommands.Drive(
                        () -> pilot.getDriveFwdPositive(),
                        () -> pilot.getDriveLeftPositive(),
                        () -> 1.0,
                        () -> pilot.getFieldOriented(), // true is field oriented
                        () -> true)
                .withName("Swerve.PilotFullTurnDrive")
                .withTimeout(2);
    }

    public static Command headingLockDrive() {
        return SwerveCommands.headingLock(
                        () -> pilot.getDriveFwdPositive(),
                        () -> pilot.getDriveLeftPositive(),
                        () -> pilot.getFieldOriented(), // true is field oriented
                        () -> true)
                .withName("Swerve.PilotHeadingLockDrive");
    }

    public static Command aimToSpeaker() {
        return SwerveCommands.aimDrive(
                        () -> pilot.getDriveFwdPositive(),
                        () -> pilot.getDriveLeftPositive(),
                        () -> Robot.vision.getAdjustedThetaToSpeaker(),
                        () -> pilot.getFieldOriented(), // true is field oriented
                        () -> true)
                .withName("Swerve.aimToRedSpeaker");
    }

    public static Command aimToFeed() {
        return SwerveCommands.aimDrive(
                        () -> pilot.getDriveFwdPositive(),
                        () -> pilot.getDriveLeftPositive(),
                        () -> Robot.vision.getAdjustedThetaToFeeder(),
                        () -> pilot.getFieldOriented(), // true is field oriented
                        () -> true)
                .withName("Swerve.aimToRedSpeaker");
    }

    public static Command alignToAmp() {
        return SwerveCommands.AlignXaimDrive(
                () -> Field.flipXifRed(Field.ampCenter.getX()),
                () -> pilot.getDriveLeftPositive(),
                () -> Math.toRadians(Field.flipAngleIfBlue(270)), // Face the back to the amp
                () -> true,
                () -> true);
    }

    public static Command turnToAmp() {
        return SwerveCommands.aimDrive(
                () -> pilot.getDriveFwdPositive(),
                () -> pilot.getDriveLeftPositive(),
                () -> Units.degreesToRadians(Field.flipAngleIfBlue(270)),
                () -> pilot.getFieldOriented(), // true is field oriented
                () -> true);
    }

    public static Command alignToAmpClimb() {
        return SwerveCommands.AlignAimDrive(
                () -> Field.Stage.ampClimb.getX(),
                () -> Field.Stage.ampClimb.getY(),
                () -> Field.Stage.ampClimb.getRotation().getRadians(),
                () -> true,
                () -> true);
    }

    // Manual Aiming Drive, no vision/pose used for these commands
    public static Command podiumAimingDrive() {
        return SwerveCommands.aimDrive(
                        () -> pilot.getDriveFwdPositive(),
                        () -> pilot.getDriveLeftPositive(),
                        () -> Units.degreesToRadians(Field.flipAngleIfBlue(Field.podiumAimAngle)),
                        () -> pilot.getFieldOriented(), // true is field oriented
                        () -> true)
                .withName("Swerve.PilotSpeakerAimingDrive");
    }

    public static Command fromAmpAimingDrive() {
        return SwerveCommands.aimDrive(
                        () -> pilot.getDriveFwdPositive(),
                        () -> pilot.getDriveLeftPositive(),
                        () -> Units.degreesToRadians(Field.flipAngleIfBlue(Field.ampAimAngle)),
                        () -> pilot.getFieldOriented(), // true is field oriented
                        () -> true)
                .withName("Swerve.PilotSpeakerAimingDrive");
    }

    public static Command ampWingAimingDrive() {
        return SwerveCommands.aimDrive(
                        () -> pilot.getDriveFwdPositive(),
                        () -> pilot.getDriveLeftPositive(),
                        () -> Units.degreesToRadians(Field.flipAngleIfBlue(Field.ampWingAimAngle)),
                        () -> pilot.getFieldOriented(), // true is field oriented
                        () -> true)
                .withName("Swerve.PilotSpeakerAimingDrive");
    }

    /**
     * Drive the robot using left stick and control orientation using the right stick Only Cardinal
     * directions are allowed
     *
     * @return
     */
    public static Command stickSteerDrive() {
        return SwerveCommands.aimDrive(
                        () -> pilot.getDriveFwdPositive(),
                        () -> pilot.getDriveLeftPositive(),
                        () -> pilot.chooseCardinalDirections(),
                        () -> pilot.getFieldOriented(), // true is field oriented
                        () -> true)
                .withName("Swerve.PilotStickSteer");
    }

    /**
     * Command that can be used to turn on the slow mode. Slow mode modifies the fwd, left, and CCW
     * methods, we don't want these to require the pilot subsystem
     */
    public static Command slowMode() {
        return Commands.startEnd(() -> pilot.setSlowMode(true), () -> pilot.setSlowMode(false));
    }

    /**
     * Command that can be used to turn on the turbo mode. Turbo mode modifies CCW methods, we don't
     * want these to require the pilot subsystem
     */
    public static Command turboMode() {
        return Commands.startEnd(() -> pilot.setTurboMode(true), () -> pilot.setTurboMode(false));
    }

    /**
     * Command that can be used to turn on the FPV mode. FPV sets field oriented or not. We don't
     * want this command to require the pilot subsystem so we use Commands.startend()
     */
    public static Command fpvMode() {
        return Commands.startEnd(
                () -> pilot.setFieldOriented(false), () -> pilot.setFieldOriented(true));
    }

    // Unused currently, for testing
    public static Command manualClimber() {
        return climber.runPercentage(() -> -pilot.controller.getRightY());
    }

    public static Command manualPivot() {
        return pivot.runManualOutput(() -> -pilot.controller.getRightY() * 0.5);
    }

    // Unused currently, for testing
    public static Command manualIntake() {
        return Robot.intake.runManualOutput(() -> -pilot.controller.getRightY());
    }
}
