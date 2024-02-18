package frc.robot.swerve.configs;

import edu.wpi.first.math.util.Units;
import frc.spectrumLib.swerve.config.DefaultConfig;
import frc.spectrumLib.swerve.config.DefaultConfig.SlotGains;
import frc.spectrumLib.swerve.config.ModuleConfig;
import frc.spectrumLib.swerve.config.ModuleConfig.SwerveModuleSteerFeedbackType;
import frc.spectrumLib.swerve.config.SwerveConfig;

public class ALPHA2024 {

    // Angle Offsets: from cancoder Absolute Position No Offset, opposite sign
    private static final double kFrontLeftCANcoderOffset = 0.336426;
    private static final double kFrontRightCANncoderOffset = -0.031006;
    private static final double kBackLeftCANcoderOffset = -0.323730;
    private static final double kBackRightCANcoderOffset = 0.492188;

    // Physical Config
    private static final double wheelBaseInches = 21.5;

    private static final double frontWheelBaseInches = 11.875;
    private static final double backWheelBaseInches = 8.375;
    private static final double trueWheelBaseInches = frontWheelBaseInches + backWheelBaseInches;
    private static final double trackWidthInches = 11.875;
    private static final double kDriveGearRatio = 6.746;
    private static final double kSteerGearRatio = 21.428;

    // Tuning Config
    // Estimated at first, then fudge-factored to make odom match record
    private static final double kWheelRadiusInches = 2;
    private static final double speedAt12VoltsMps = 6;

    private static final double slipCurrent = 80;
    private static final SlotGains steerGains = new SlotGains(100, 0, 0.05, 0, 0);
    private static final SlotGains driveGains = new SlotGains(0.4, 0, 0, 0, 0);

    /*Rotation Controller*/
    private static final double kPRotationController = 7.0;
    private static final double kIRotationController = 0.0;
    private static final double kDRotationController = 0.0;

    /*Profiling Configs*/
    private static final double maxVelocity = speedAt12VoltsMps;
    private static final double maxAccel = maxVelocity * 1.5; // take 1/2 sec to get to max speed.
    private static final double maxAngularVelocity =
            maxVelocity
                    / Units.inchesToMeters(
                            Math.hypot(wheelBaseInches / 2.0, trackWidthInches / 2.0));
    private static final double maxAngularAcceleration = Math.pow(maxAngularVelocity, 2);
    private static final double deadband = 0.1;
    private static final double rotationDeadband = 0.1;

    // Device Setup
    private static final String kCANbusName = "3847";
    private static final boolean supportsPro = true;
    private static final SwerveModuleSteerFeedbackType steerFeedbackType =
            SwerveModuleSteerFeedbackType.FusedCANcoder;

    // Wheel Positions
    private static final double kFrontLeftXPos = Units.inchesToMeters(frontWheelBaseInches);
    private static final double kFrontLeftYPos = Units.inchesToMeters(trackWidthInches);
    private static final double kFrontRightXPos = Units.inchesToMeters(frontWheelBaseInches);
    private static final double kFrontRightYPos = Units.inchesToMeters(-trackWidthInches);
    private static final double kBackLeftXPos = Units.inchesToMeters(-backWheelBaseInches);
    private static final double kBackLeftYPos = Units.inchesToMeters(trackWidthInches);
    private static final double kBackRightXPos = Units.inchesToMeters(-backWheelBaseInches);
    private static final double kBackRightYPos = Units.inchesToMeters(-trackWidthInches);

    public static final ModuleConfig FrontLeft =
            DefaultConfig.FrontLeft.withCANcoderOffset(kFrontLeftCANcoderOffset)
                    .withLocationX(kFrontLeftXPos)
                    .withLocationY(kFrontLeftYPos)
                    .withSlipCurrent(slipCurrent)
                    .withSpeedAt12VoltsMps(speedAt12VoltsMps)
                    .withDriveMotorGearRatio(kDriveGearRatio)
                    .withSteerMotorGearRatio(kSteerGearRatio)
                    .withDriveMotorGains(driveGains)
                    .withSteerMotorGains(steerGains)
                    .withWheelRadius(kWheelRadiusInches)
                    .withFeedbackSource(steerFeedbackType);

    public static final ModuleConfig FrontRight =
            DefaultConfig.FrontRight.withCANcoderOffset(kFrontRightCANncoderOffset)
                    .withLocationX(kFrontRightXPos)
                    .withLocationY(kFrontRightYPos)
                    .withSlipCurrent(slipCurrent)
                    .withSpeedAt12VoltsMps(speedAt12VoltsMps)
                    .withDriveMotorGearRatio(kDriveGearRatio)
                    .withSteerMotorGearRatio(kSteerGearRatio)
                    .withDriveMotorGains(driveGains)
                    .withSteerMotorGains(steerGains)
                    .withWheelRadius(kWheelRadiusInches)
                    .withFeedbackSource(steerFeedbackType)
            //     .withDriveMotorInverted(false)
            ;

    public static final ModuleConfig BackLeft =
            DefaultConfig.BackLeft.withCANcoderOffset(kBackLeftCANcoderOffset)
                    .withLocationX(kBackLeftXPos)
                    .withLocationY(kBackLeftYPos)
                    .withSlipCurrent(slipCurrent)
                    .withSpeedAt12VoltsMps(speedAt12VoltsMps)
                    .withDriveMotorGearRatio(kDriveGearRatio)
                    .withSteerMotorGearRatio(kSteerGearRatio)
                    .withDriveMotorGains(driveGains)
                    .withSteerMotorGains(steerGains)
                    .withWheelRadius(kWheelRadiusInches)
                    .withFeedbackSource(steerFeedbackType);

    public static final ModuleConfig BackRight =
            DefaultConfig.BackRight.withCANcoderOffset(kBackRightCANcoderOffset)
                    .withLocationX(kBackRightXPos)
                    .withLocationY(kBackRightYPos)
                    .withSlipCurrent(slipCurrent)
                    .withSpeedAt12VoltsMps(speedAt12VoltsMps)
                    .withDriveMotorGearRatio(kDriveGearRatio)
                    .withSteerMotorGearRatio(kSteerGearRatio)
                    .withDriveMotorGains(driveGains)
                    .withSteerMotorGains(steerGains)
                    .withWheelRadius(kWheelRadiusInches)
                    .withFeedbackSource(steerFeedbackType);

    public static final ModuleConfig[] ModuleConfigs = {FrontLeft, FrontRight, BackLeft, BackRight};

    public static final SwerveConfig config =
            DefaultConfig.DrivetrainConstants.withCANbusName(kCANbusName)
                    .withSupportsPro(supportsPro)
                    .withModules(ModuleConfigs)
                    .withRotationGains(
                            kPRotationController, kIRotationController, kDRotationController)
                    .withProfilingConfigs(
                            maxVelocity, maxAccel, maxAngularVelocity, maxAngularAcceleration)
                    .withDeadbandConfig(deadband, rotationDeadband);
}
