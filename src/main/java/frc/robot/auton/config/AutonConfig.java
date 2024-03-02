package frc.robot.auton.config;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.auton.Auton;
import frc.robot.swerve.commands.ApplyChassisSpeeds;
import java.util.Optional;

public class AutonConfig {

    public static final double kTranslationP = 5;
    public static final double kTranslationI = 0.0;
    public static final double kTranslationD = 0.0;
    public static final double kRotationP = 5;
    public static final double kRotationI = 0.0;
    public static final double kRotationD = 0.0;
    public static final double maxModuleSpeed = 4.5;
    public static final double driveBaseRadius = 0.4;

    public static boolean commandInit = false;

    public static HolonomicPathFollowerConfig AutonPathFollowerConfig =
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely
                    // live in your Constants class
                    new PIDConstants(
                            kTranslationP,
                            kTranslationI,
                            kTranslationD), // Translation PID constants
                    new PIDConstants(kRotationP, kRotationI, kRotationD), // Rotation PID constants
                    maxModuleSpeed, // Max module speed, in m/s
                    driveBaseRadius, // Drive base radius in meters. Distance from robot center to
                    // furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for
                    // the options here
                    );

    public static void configureAutoBuilder() {
        // All other subsystem initialization
        // ...

        // Set the method that will be used to get rotation overrides
        PPHolonomicDriveController.setRotationTargetOverride(() -> getRotationTargetOverride());

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                Robot.swerve::getPose, // Robot pose supplier
                Robot.swerve::resetPose, // Method to reset odometry (will be called if your auto
                // has a starting pose)
                Robot.swerve::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT
                // RELATIVE
                ApplyChassisSpeeds.robotRelativeOutput(
                        false), // Method that will drive the robot given ROBOT
                // RELATIVE
                // ChassisSpeeds
                AutonConfig.AutonPathFollowerConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    Optional<Alliance> alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                Robot.swerve // Reference to this subsystem to set requirements
                );
    }

    public static Optional<Rotation2d> getRotationTargetOverride() {
        // Some condition that should decide if we want to override rotation
        if (Auton.trackNote) {
            // Return an optional containing the rotation override (this should be a field relative
            // rotation)
            if (Robot.vision.noteInView()) {
                return Optional.of(Rotation2d.fromDegrees(Robot.vision.getOffsetToNote()));
            } else {
                return Optional.empty();
            }
        } else if (Auton.trackSpeaker) {
            if (Robot.vision.speakerInView()) {
                return Optional.of(Rotation2d.fromDegrees(Robot.vision.getOffsetToSpeaker()));
            } else {
                return Optional.empty();
            }
        } else {
            // return an empty optional when we don't want to override the path's rotation
            return Optional.empty();
        }
    }
}
