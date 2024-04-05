package frc.robot.swerve.commands;

import edu.wpi.first.math.geometry.Pose2d;
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
        // Wait a little before enabling heading lock, allows any turns to finish
        swerve.setDefaultCommand(
                PilotCommands.pilotDrive()
                        // .withTimeout(0.5)
                        // .andThen(PilotCommands.headingLockDrive())
                        .ignoringDisable(true)
                        .withName("SwerveCommands.default"));
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

    public static Command AlignXdrive(
            DoubleSupplier targetX,
            DoubleSupplier velocityY,
            DoubleSupplier rotationalRate,
            BooleanSupplier isFieldOriented,
            BooleanSupplier isOpenLoop) {
        return resetAlignmentControllers()
                .andThen(
                        Drive(
                                () -> swerve.calculateXController(targetX),
                                velocityY,
                                rotationalRate,
                                isFieldOriented,
                                isOpenLoop))
                .withName("Swerve.AlignXdrive");
    };

    public static Command AlignYdrive(
            DoubleSupplier velocityX,
            DoubleSupplier targetY,
            DoubleSupplier rotationalRate,
            BooleanSupplier isFieldOriented,
            BooleanSupplier isOpenLoop) {
        return resetAlignmentControllers()
                .andThen(
                        Drive(
                                velocityX,
                                () -> swerve.calculateYController(targetY),
                                rotationalRate,
                                isFieldOriented,
                                isOpenLoop))
                .withName("Swerve.AlignYdrive");
    };

    public static Command AlignDrive(
            DoubleSupplier targetX,
            DoubleSupplier targetY,
            DoubleSupplier rotationalRate,
            BooleanSupplier isFieldOriented,
            BooleanSupplier isOpenLoop) {
        return resetAlignmentControllers()
                .andThen(
                        Drive(
                                () -> swerve.calculateXController(targetX),
                                () -> swerve.calculateYController(targetY),
                                rotationalRate,
                                isFieldOriented,
                                isOpenLoop))
                .withName("Swerve.AlignDrive");
    }

    public static Command AlignXaimDrive(
            DoubleSupplier targetX,
            DoubleSupplier velocityY,
            DoubleSupplier targetRadians,
            BooleanSupplier isFieldOriented,
            BooleanSupplier isOpenLoop) {
        return resetAlignmentControllers()
                .andThen(
                        aimDrive(
                                () -> swerve.calculateXController(targetX),
                                velocityY,
                                targetRadians,
                                isFieldOriented,
                                isOpenLoop))
                .withName("Swerve.AlignXaimDrive");
    }

    public static Command AlignYaimDrive(
            DoubleSupplier velocityX,
            DoubleSupplier targetY,
            DoubleSupplier targetRadians,
            BooleanSupplier isFieldOriented,
            BooleanSupplier isOpenLoop) {
        return resetAlignmentControllers()
                .andThen(
                        aimDrive(
                                velocityX,
                                () -> swerve.calculateYController(targetY),
                                targetRadians,
                                isFieldOriented,
                                isOpenLoop))
                .withName("Swerve.AlignYaimDrive");
    }

    public static Command AlignAimDrive(
            DoubleSupplier targetX,
            DoubleSupplier targetY,
            DoubleSupplier targetRadians,
            BooleanSupplier isFieldOriented,
            BooleanSupplier isOpenLoop) {
        return resetAlignmentControllers()
                .andThen(
                        aimDrive(
                                () -> swerve.calculateXController(targetX),
                                () -> swerve.calculateYController(targetY),
                                targetRadians,
                                isFieldOriented,
                                isOpenLoop))
                .withName("Swerve.AlignAimDrive");
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

    public static Command resetAlignmentControllers() {
        return swerve.runOnce(() -> swerve.resetAlignmentControllers())
                .withName("ResetAlignmentControllers");
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

    public static Command cardinalReorient() {
        return swerve.runOnce(() -> swerve.cardinalReorient()).withName("Swerve.cardinalReorient");
    }

    /**
     * Temporarily sets the swerve modules to coast mode. The configuration is applied when the
     * command is started and reverted when the command is ended.
     */
    public static Command coastMode() {
        return swerve.startEnd(() -> swerve.setCoastMode(), () -> swerve.setBrakeMode())
                .ignoringDisable(true)
                .withName("Swerve.coastMode");
    }

    public static Command resetPose(Pose2d pose) {
        return swerve.runOnce(() -> swerve.resetPose(pose)).withName("SwerveCommands.resetPose");
    }

    public static Command getSwerveSwitch() {
        return new Command() {
            boolean positiveSpeed;
            boolean started = false;

            // constructor
            {
            }

            @Override
            public void initialize() {
                double speed = swerve.getVelocity(true).vxMetersPerSecond;
                positiveSpeed = speed >= 0;
                started = true;
            }

            @Override
            public void execute() {}

            @Override
            public void end(boolean interrupted) {
                started = false;
            }

            @Override
            public boolean isFinished() {
                if (started) {
                    double speed = swerve.getVelocity(true).vxMetersPerSecond;
                    return speed >= 0 != positiveSpeed;
                } else {
                    return false;
                }
            }
        }.withName("Swerve.getSwerveSwitch");
    }
}
