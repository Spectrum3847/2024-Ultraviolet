package frc.robot.pilot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
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
        pilot.setDefaultCommand(rumble(0, 99999).repeatedly().withName("Pilot.default"));
    }

    /** Command that can be used to rumble the pilot controller */
    public static Command rumble(double intensity, double durationSeconds) {
        return pilot.rumbleCommand(intensity, durationSeconds);
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

    public static Command aimToRedSpeaker() {
        return SwerveCommands.aimDrive(
                        () -> pilot.getDriveFwdPositive(),
                        () -> pilot.getDriveLeftPositive(),
                        () ->
                                // Units.degreesToRadians(
                                Robot.swerve.getRotation().getRadians()
                                        + Robot.vision.getThetaToHybrid() // or
                        // Robot.swerve.getRotation()?
                        ,
                        () -> pilot.getFieldOriented(), // true is field oriented
                        () -> true)
                .withName("Swerve.aimToRedSpeaker");
    }

    public static Command noteAimingDrive() {
        return SwerveCommands.aimDrive(
                        () -> pilot.getDriveFwdPositive(),
                        () -> pilot.getDriveLeftPositive(),
                        () -> (Units.degreesToRadians(Robot.vision.getOffsetToNote())),
                        () -> pilot.getFieldOriented(), // true is field oriented
                        () -> true)
                .withName("Swerve.PilotNoteAimingDrive");
    }

    public static Command speakerAimingDrive() {
        return SwerveCommands.aimDrive(
                        () -> pilot.getDriveFwdPositive(),
                        () -> pilot.getDriveLeftPositive(),
                        () -> (Units.degreesToRadians(Robot.vision.getOffsetToSpeaker())),
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

    public static Command manualClimber() {
        return climber.runPercentage(() -> -pilot.controller.getRightY());
    }

    public static Command manualPivot() {
        return pivot.runManualOutput(() -> -pilot.controller.getRightY() * 0.5);
    }

    public static Command manualIntake() {
        return Robot.intake.runManualOutput(() -> -pilot.controller.getRightY());
    }
}
