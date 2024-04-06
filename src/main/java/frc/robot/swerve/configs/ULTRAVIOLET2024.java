package frc.robot.swerve.configs;

import edu.wpi.first.math.util.Units;
import frc.robot.RobotConfig;
import frc.robot.mechanisms.pivot.Pivot.CANCoderFeedbackType;
import frc.spectrumLib.swerve.config.DefaultConfig;
import frc.spectrumLib.swerve.config.DefaultConfig.SlotGains;
import frc.spectrumLib.swerve.config.ModuleConfig;
import frc.spectrumLib.swerve.config.ModuleConfig.SwerveModuleSteerFeedbackType;
import frc.spectrumLib.swerve.config.SwerveConfig;

/** Config for Comp */
public class ULTRAVIOLET2024 {

    // Angle Offsets: from cancoder Absolute Position No Offset, opposite sign
    private static final double kFrontLeftCANcoderOffset = 0.044189; // 0.044189
    private static final double kFrontRightCANncoderOffset = 0.225098; // 0.224609
    private static final double kBackLeftCANcoderOffset = -0.304443; // -0.304932
    private static final double kBackRightCANcoderOffset = -0.261475; // -0.260986

    // Physical Config
    private static final double frontWheelBaseInches = 11.875;
    private static final double backWheelBaseInches = 8.375;
    private static final double trueWheelBaseInches = frontWheelBaseInches + backWheelBaseInches;
    private static final double trackWidthInches = 11.875;
    private static final double kDriveGearRatio = 5.35714285714;
    private static final double kSteerGearRatio = 21.4285714286;

    // Tuning Config
    // Estimated at first, then fudge-factored to make odom match record
    private static final double kWheelRadiusInches = 3.815 / 2; // Updated for VexIQ Pro Wheels
    private static final double speedAt12VoltsMps = 6;

    private static final double slipCurrent = 80;
    private static final double supplyCurrentLimit = 75;
    private static final double supplyCurrentThreshold = 75;
    private static final double peakForwardTorqueCurrent = 300;
    private static final double peakReverseTorqueCurrent = 300;
    private static final SlotGains steerGains = new SlotGains(100, 0, 0, 0, 0);
    private static final SlotGains driveGains = new SlotGains(8, 0, 0.1, 0, 0.8);

    /*Rotation Controller*/
    private static final double kPRotationController = 8.0;
    private static final double kIRotationController = 0.0;
    private static final double kDRotationController = 0.0;

    /*Alignment Controllers */
    private static final double kPAlignmentController = 12.0;
    private static final double kIAlignmentController = 0.0;
    private static final double kDAlignmentController = 0.0;

    /*Profiling Configs*/
    private static final double maxVelocity = speedAt12VoltsMps;
    private static final double maxAccel = maxVelocity * 1.5; // take 1/2 sec to get to max speed.
    private static final double maxAngularVelocity =
            maxVelocity / Units.inchesToMeters(Math.hypot(trueWheelBaseInches, trackWidthInches));
    private static final double maxAngularAcceleration = Math.pow(maxAngularVelocity, 2);
    private static final double deadband = 0.1;
    private static final double rotationDeadband = 0.1;

    // Device Setup
    private static final String kCANbusName = RobotConfig.CANIVORE;
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

    // Pivot CANCoder configs
    public static final double pivotCANcoderOffset = -0.05; // flip sign
    public static final CANCoderFeedbackType pivotFeedbackSource =
            CANCoderFeedbackType.FusedCANcoder;

    public static final ModuleConfig FrontLeft =
            DefaultConfig.FrontLeft.withCANcoderOffset(kFrontLeftCANcoderOffset)
                    .withLocationX(kFrontLeftXPos)
                    .withLocationY(kFrontLeftYPos)
                    .withSlipCurrent(slipCurrent)
                    .withSupplyCurrent(supplyCurrentLimit, supplyCurrentThreshold)
                    .withForwardTorqueCurrentLimit(peakForwardTorqueCurrent)
                    .withReverseTorqueCurrentLimit(peakReverseTorqueCurrent)
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
                    .withSupplyCurrent(supplyCurrentLimit, supplyCurrentThreshold)
                    .withForwardTorqueCurrentLimit(peakForwardTorqueCurrent)
                    .withReverseTorqueCurrentLimit(peakReverseTorqueCurrent)
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
                    .withSupplyCurrent(supplyCurrentLimit, supplyCurrentThreshold)
                    .withForwardTorqueCurrentLimit(peakForwardTorqueCurrent)
                    .withReverseTorqueCurrentLimit(peakReverseTorqueCurrent)
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
                    .withSupplyCurrent(supplyCurrentLimit, supplyCurrentThreshold)
                    .withForwardTorqueCurrentLimit(peakForwardTorqueCurrent)
                    .withReverseTorqueCurrentLimit(peakReverseTorqueCurrent)
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
                    .withAlignmentGains(
                            kPAlignmentController, kIAlignmentController, kDAlignmentController)
                    .withProfilingConfigs(
                            maxVelocity, maxAccel, maxAngularVelocity, maxAngularAcceleration)
                    .withDeadbandConfig(deadband, rotationDeadband)
                    .withPivotConfig(pivotCANcoderOffset, pivotFeedbackSource);
}
