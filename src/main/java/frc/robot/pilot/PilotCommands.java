package frc.robot.pilot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;
import frc.robot.swerve.commands.SwerveCommands;

/** This class should have any command calls that directly call the Pilot */
public class PilotCommands {
    private static Pilot pilot = Robot.pilot;

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
                        () -> pilot.getDriveCCWPositive(),
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
                        () -> pilot.getRightStickCardinals(),
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
     * Command that can be used to turn on the FPV mode. FPV sets field oriented or not. We don't
     * want this command to require the pilot subsystem so we use Commands.startend()
     */
    public static Command fpvMode() {
        return Commands.startEnd(
                () -> pilot.setFieldOriented(false), () -> pilot.setFieldOriented(true));
    }

    // TODO: temporary manual commands for mechanisms; usage will change

    public static Command manualAmpTrap() {
        return new RunCommand(
                        () -> Robot.ampTrap.runPercentage(pilot.controller.getRightY()),
                        Robot.ampTrap)
                .withName("AmpTrap.manualOutput");
    }

    public static Command manualClimber() {
        return new RunCommand(
                        () -> Robot.climber.runPercentage(pilot.controller.getRightY()),
                        Robot.climber)
                .withName("Climber.manualOutput");
    }

    public static Command manualElevator() {
        return new RunCommand(() -> Robot.elevator.runPercentage(0.3), Robot.elevator)
                .withName("Elevator.manualOutput");
    }

    public static Command manualFeeder() {
        return new RunCommand(
                        () -> Robot.feeder.runPercentage(pilot.controller.getRightY()),
                        Robot.feeder)
                .withName("Feeder.manualOutput");
    }

    public static Command manualIntake() {
        return new RunCommand(
                        () -> Robot.intake.runPercentage(pilot.controller.getRightY()),
                        Robot.intake)
                .withName("Intake.manualOutput");
    }

    public static Command manualPivot() {
        return new RunCommand(
                        () -> Robot.pivot.runManualOutput(pilot.controller.getRightY()),
                        Robot.pivot)
                .withName("Pivot.manualOutput");
    }

    public static Command manualLauncher() {
        return new RunCommand(
                        () -> {
                            Robot.leftLauncher.runPercentage(pilot.controller.getRightY());
                            Robot.rightLauncher.runPercentage(pilot.controller.getRightY());
                        },
                        Robot.leftLauncher,
                        Robot.rightLauncher)
                .withName("Launcher.manualOutput");
    }
}
