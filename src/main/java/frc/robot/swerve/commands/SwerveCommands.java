package frc.robot.swerve.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.pilot.PilotCommands;
import frc.robot.swerve.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveCommands {
    private static Swerve swerve = Robot.swerve;
    /** Velocity deadband applied to the swerve control request */
    private static DoubleSupplier requestDeadband =
            () -> (swerve.config.deadband * swerve.config.maxVelocity);

    public static void setupDefaultCommand() {
        swerve.setDefaultCommand(PilotCommands.pilotDrive());
    }

    /** Turn the swerve wheels to an X to prevent the robot from moving */
    public static Command Xbrake() {
        return Xbrake.run().withName("Swerve.Xbrake");
    }

    /** Drive the swerve */
    public static Command Drive(
            DoubleSupplier velocityX,
            DoubleSupplier velocityY,
            DoubleSupplier rotationalRate,
            BooleanSupplier isFieldOriented,
            BooleanSupplier isOpenLoop) {
        return Drive.run(
                        velocityX,
                        velocityY,
                        rotationalRate,
                        requestDeadband,
                        isFieldOriented,
                        isOpenLoop)
                .withName("Swerve.Drive")
                .ignoringDisable(true);
    }

    /**
     * Reset the turn controller and then run the drive command with a angle supplier. This can be
     * used for aiming at a goal or heading locking, etc
     */
    public static Command aimDrive(
            DoubleSupplier velocityX,
            DoubleSupplier velocityY,
            DoubleSupplier targetRadians,
            BooleanSupplier isFieldOriented,
            BooleanSupplier isOpenLoop) {
        return resetTurnController()
                .andThen(
                        Drive(
                                velocityX,
                                velocityY,
                                () -> swerve.calculateRotationController(targetRadians),
                                isFieldOriented,
                                isOpenLoop))
                .withName("Swerve.AimDrive");
    }

    /**
     * Reset the turn controller, set the target heading to the current heading(end that command
     * immediately), and then run the drive command with the Rotation controller. The rotation
     * controller will only engague if you are driving x or y.
     */
    public static Command headingLock(
            DoubleSupplier velocityX,
            DoubleSupplier velocityY,
            BooleanSupplier isFieldOriented,
            BooleanSupplier isOpenLoop) {
        return resetTurnController()
                .andThen(
                        setTargetHeading(() -> swerve.getRotation().getRadians()).until(() -> true),
                        Drive(
                                velocityX,
                                velocityY,
                                () -> {
                                    if (velocityX.getAsDouble() == 0
                                            && velocityY.getAsDouble() == 0) {
                                        return 0.0;
                                    } else {
                                        return swerve.calculateRotationController(
                                                () -> swerve.getTargetHeading());
                                    }
                                },
                                isFieldOriented,
                                isOpenLoop))
                .withName("Swerve.HeadingLock");
    }

    /** Apply a chassis speed to the swerve */
    public static Command ApplyChassisSpeeds(
            Supplier<ChassisSpeeds> speeds, BooleanSupplier isOpenLoop) {
        return ApplyChassisSpeeds.run(speeds, isOpenLoop).withName("Swerve.ApplyChassisSpeeds");
    }

    /** Reset the turn controller */
    public static Command resetTurnController() {
        return swerve.runOnce(() -> swerve.resetRotationController())
                .withName("ResetTurnController");
    }

    public static Command setTargetHeading(DoubleSupplier targetHeading) {
        return Commands.run(() -> swerve.setTargetHeading(targetHeading.getAsDouble()))
                .withName("SetTargetHeading");
    }

    public static Command reorient(double angle) {
        return swerve.runOnce(() -> swerve.reorient(angle)).withName("Swerve.reorient");
    }

    // do not use this
    // public static Command smartReorient(double angle) {
    //     return swerve.runOnce(
    //                     () -> {
    //                         double newAngle =
    //                                 (DriverStation.getAlliance().orElse(Alliance.Blue)
    //                                                 == Alliance.Red)
    //                                         ? angle
    //                                         : (angle + 180);
    //                         swerve.reorient(newAngle);
    //                     })
    //             .withName("Swerve.smartReorient");
    // }

    public static Command reorientForward() {
        return swerve.runOnce(() -> swerve.reorientForward()).withName("Swerve.reorientForward");
    }

    public static Command reorientLeft() {
        return swerve.runOnce(() -> swerve.reorientLeft()).withName("Swerve.reorientLeft");
    }

    public static Command reorientRight() {
        return swerve.runOnce(() -> swerve.reorientRight()).withName("Swerve.reorientRight");
    }

    public static Command reorientBack() {
        return swerve.runOnce(() -> swerve.reorientBack()).withName("Swerve.reorientBack");
    }
    // Swerve Command Options
    // - Drive needs to work with slow mode (this might be done in PilotCommands)
}
