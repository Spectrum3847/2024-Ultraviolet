package frc.robot.pilot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.crescendo.Field;
import frc.robot.Robot;
import frc.robot.leds.LEDsCommands;
import frc.robot.mechanisms.climber.Climber;
import frc.robot.mechanisms.pivot.Pivot;
import frc.robot.swerve.commands.SwerveCommands;

/** This class should have any command calls that directly call the Pilot */
public class PilotCommands {
    private static Pilot pilot = Robot.pilot;
    private static Climber climber = Robot.climber;
    private static Pivot pivot = Robot.pivot;

    /** Set default command to turn off the rumble */
    public static void setupDefaultCommand() {
        Robot.pilot.setDefaultCommand(
                rumble(0, 1)); // launchReadyRumble().withName("Pilot.default"));
    }

    /** Command that can be used to rumble the pilot controller */
    public static Command rumble(double intensity, double durationSeconds) {
        return Robot.pilot.rumbleCommand(intensity, durationSeconds);
    }

    public static Command launchReadyRumble() {
        return new InstantCommand(() -> {}, Robot.pilot);
        /*return new FunctionalCommand(
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
        */
    }

    /** Full control of the swerve by the Pilot command */
    public static Command pilotDrive() {
        return SwerveCommands.Drive(
                        () -> Robot.pilot.getDriveFwdPositive(),
                        () -> Robot.pilot.getDriveLeftPositive(),
                        () -> -Robot.pilot.getDriveCCWPositive(),
                        () -> Robot.pilot.getFieldOriented(), // true is field oriented
                        () -> true)
                .withName("Swerve.PilotDrive");
    }

    public static Command fullTurnDrive() {
        return SwerveCommands.Drive(
                        () -> Robot.pilot.getDriveFwdPositive(),
                        () -> Robot.pilot.getDriveLeftPositive(),
                        () -> 1.0,
                        () -> Robot.pilot.getFieldOriented(), // true is field oriented
                        () -> true)
                .withName("Swerve.PilotFullTurnDrive")
                .withTimeout(2);
    }

    public static Command headingLockDrive() {
        return SwerveCommands.headingLock(
                        () -> Robot.pilot.getDriveFwdPositive(),
                        () -> Robot.pilot.getDriveLeftPositive(),
                        () -> Robot.pilot.getFieldOriented(), // true is field oriented
                        () -> true)
                .withName("Swerve.PilotHeadingLockDrive");
    }

    public static Command aimToSpeaker() {
        return SwerveCommands.aimDrive(
                        () -> Robot.pilot.getDriveFwdPositive(),
                        () -> Robot.pilot.getDriveLeftPositive(),
                        () -> Robot.vision.getAdjustedThetaToSpeaker(),
                        () -> Robot.pilot.getFieldOriented(), // true is field oriented
                        () -> true)
                .withName("Swerve.aimToRedSpeaker");
    }

    public static Command aimToFeed() {
        return SwerveCommands.aimDrive(
                        () -> Robot.pilot.getDriveFwdPositive(),
                        () -> Robot.pilot.getDriveLeftPositive(),
                        () -> Robot.vision.getAdjustedThetaToFeeder(),
                        () -> Robot.pilot.getFieldOriented(), // true is field oriented
                        () -> true)
                .withName("Swerve.aimToRedSpeaker");
    }

    public static Command aimToDeepFeed() {
        return SwerveCommands.aimDrive(
                        () -> Robot.pilot.getDriveFwdPositive(),
                        () -> Robot.pilot.getDriveLeftPositive(),
                        () -> Robot.vision.getAdjustedThetaToDeepFeeder(),
                        () -> Robot.pilot.getFieldOriented(), // true is field oriented
                        () -> true)
                .withName("Swerve.aimToRedSpeaker");
    }

    public static Command aimToManualFeed() {
        return SwerveCommands.aimDrive(
                        () -> Robot.pilot.getDriveFwdPositive(),
                        () -> Robot.pilot.getDriveLeftPositive(),
                        () -> Units.degreesToRadians(Field.flipAngleIfBlue(300)),
                        () -> Robot.pilot.getFieldOriented(), // true is field oriented
                        () -> true)
                .withName("Swerve.aimToRedSpeaker");
    }

    public static Command alignToAmp() {
        return SwerveCommands.AlignXaimDrive(
                () -> Field.flipXifRed(Field.ampCenter.getX()),
                () -> Robot.pilot.getDriveLeftPositive(),
                () -> Math.toRadians(Field.flipAngleIfBlue(270)), // Face the back to the amp
                () -> true,
                () -> true);
    }

    public static Command turnLaunchToAmp() {
        return SwerveCommands.aimDrive(
                () -> Robot.pilot.getDriveFwdPositive(),
                () -> Robot.pilot.getDriveLeftPositive(),
                () -> Units.degreesToRadians(Field.flipAngleIfBlue(90)),
                () -> Robot.pilot.getFieldOriented(), // true is field oriented
                () -> true);
    }

    public static Command alignToAmpClimb() {
        return SwerveCommands.AlignYaimDrive(
                () -> Field.Stage.centerClimb.getX(),
                () -> Field.Stage.ampClimb.getY(),
                () -> Field.Stage.ampClimb.getRotation().getRadians(),
                () -> true,
                () -> true);
    }

    public static Command alignToCenterClimb() {
        return SwerveCommands.AlignYaimDrive(
                () -> Robot.pilot.getDriveFwdPositive(),
                () -> Field.Stage.centerClimb.getY(),
                () -> Field.Stage.centerClimb.getRotation().getRadians(),
                () -> true,
                () -> true);
    }

    // Manual Aiming Drive, no vision/pose used for these commands
    public static Command podiumAimingDrive() {
        return SwerveCommands.aimDrive(
                        () -> Robot.pilot.getDriveFwdPositive(),
                        () -> Robot.pilot.getDriveLeftPositive(),
                        () -> Units.degreesToRadians(Field.flipAngleIfBlue(Field.podiumAimAngle)),
                        () -> Robot.pilot.getFieldOriented(), // true is field oriented
                        () -> true)
                .withName("Swerve.PilotSpeakerAimingDrive");
    }

    public static Command fromAmpAimingDrive() {
        return SwerveCommands.aimDrive(
                        () -> Robot.pilot.getDriveFwdPositive(),
                        () -> Robot.pilot.getDriveLeftPositive(),
                        () -> Units.degreesToRadians(Field.flipAngleIfBlue(Field.ampAimAngle)),
                        () -> Robot.pilot.getFieldOriented(), // true is field oriented
                        () -> true)
                .withName("Swerve.PilotSpeakerAimingDrive");
    }

    public static Command ampWingAimingDrive() {
        return SwerveCommands.aimDrive(
                        () -> Robot.pilot.getDriveFwdPositive(),
                        () -> Robot.pilot.getDriveLeftPositive(),
                        () -> Units.degreesToRadians(Field.flipAngleIfBlue(Field.ampWingAimAngle)),
                        () -> Robot.pilot.getFieldOriented(), // true is field oriented
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
                        () -> Robot.pilot.getDriveFwdPositive(),
                        () -> Robot.pilot.getDriveLeftPositive(),
                        () -> Robot.pilot.chooseCardinalDirections(),
                        () -> Robot.pilot.getFieldOriented(), // true is field oriented
                        () -> true)
                .withName("Swerve.PilotStickSteer");
    }

    /**
     * Turn to climb locations idk what i'm doing probably delete :) Aims at climb location close to
     * center field
     *
     * @return
     */
    public static Command aimToClimbLeft() {
        return SwerveCommands.aimDrive(
                        () -> Robot.pilot.getDriveFwdPositive(),
                        () -> Robot.pilot.getDriveLeftPositive(),
                        () -> (Units.degreesToRadians(Field.flipAngleIfBlue(-60))), // -240
                        () -> Robot.pilot.getFieldOriented(), // tgfrue is field oriented
                        () -> true)
                .withName("Swerve.aimToClimbLeft");
    }

    public static Command aimToClimbRight() {
        return SwerveCommands.aimDrive(
                        () -> Robot.pilot.getDriveFwdPositive(),
                        () -> Robot.pilot.getDriveLeftPositive(),
                        () -> (Units.degreesToRadians(Field.flipAngleIfBlue(60))), // -120
                        () -> Robot.pilot.getFieldOriented(), // true is field oriented
                        () -> true)
                .withName("Swerve.aimToClimbRight");
    }

    public static Command aimToClimbBack() {
        return SwerveCommands.aimDrive(
                        () -> Robot.pilot.getDriveFwdPositive(),
                        () -> Robot.pilot.getDriveLeftPositive(),
                        () -> (Units.degreesToRadians(Field.flipAngleIfBlue(180))), // 0
                        () -> Robot.pilot.getFieldOriented(), // true is field oriented
                        () -> true)
                .withName("Swerve.aimToClimbBack");
    }

    public static Command turnToAmp() {
        return SwerveCommands.aimDrive(
                () -> Robot.pilot.getDriveFwdPositive(),
                () -> Robot.pilot.getDriveLeftPositive(),
                () -> Units.degreesToRadians(Field.flipAngleIfBlue(90)),
                () -> Robot.pilot.getFieldOriented(), // true is field oriented
                () -> true);
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
        return Commands.startEnd(() -> pilot.setTurboMode(true), () -> pilot.setTurboMode(false))
                .alongWith(LEDsCommands.strobeOrangeLED());
    }

    /**
     * Command that can be used to turn on the FPV mode. FPV sets field oriented or not. We don't
     * want this command to require the pilot subsystem so we use Commands.startend()
     */
    public static Command fpvMode() {
        return Commands.startEnd(
                () -> Robot.pilot.setFieldOriented(false), () -> pilot.setFieldOriented(true));
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
