package frc.spectrumLib.swerve.config;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.util.Units;
import frc.spectrumLib.swerve.config.ModuleConfig.SwerveModuleSteerFeedbackType;

public class DefaultConfig {

    // Angle Offsets
    private static final double kFrontLeftCANcoderOffset = 0;
    private static final double kFrontRightCANncoderOffset = 0;
    private static final double kBackLeftCANcoderOffset = 0;
    private static final double kBackRightCANcoderOffset = 0;

    // Tuning Config
    // Estimated at first, then fudge-factored to make odom match record
    private static final double kWheelRadiusInches = 2;
    private static final double speedAt12VoltsMps = 6; // Units.feetToMeters(16);
    private static final double slipCurrent = 800;
    private static final double peakForwardTorqueCurrent = 300;
    private static final double PeakReverseTorqueCurrent = 300;
    private static final SlotGains steerGains = new SlotGains(100, 0, 0.05, 0, 0);
    private static final SlotGains driveGains = new SlotGains(0.4, 0, 0, 0, 0);

    /*Rotation Controller*/
    private static final double kPRotationController = 0.0;
    private static final double kIRotationController = 0.0;
    private static final double kDRotationController = 0.0;

    // Device Setup
    private static final String kCANbusName = "*"; // canivore
    private static final boolean supportsPro = false;
    private static final SwerveModuleSteerFeedbackType steerFeedbackType =
            SwerveModuleSteerFeedbackType.RemoteCANcoder;

    private static final int kPigeonId = 0;
    private static final int kFrontLeftDriveMotorId = 1;
    private static final int kFrontLeftSteerMotorId = 2;
    private static final int kFrontLeftEncoderId = 3;
    private static final int kFrontRightDriveMotorId = 11;
    private static final int kFrontRightSteerMotorId = 12;
    private static final int kFrontRightEncoderId = 13;
    private static final int kBackLeftDriveMotorId = 21;
    private static final int kBackLeftSteerMotorId = 22;
    private static final int kBackLeftEncoderId = 23;
    private static final int kBackRightDriveMotorId = 31;
    private static final int kBackRightSteerMotorId = 32;
    private static final int kBackRightEncoderId = 33;

    // Physical Config
    private static final double wheelBaseInches = 21.5;
    private static final double trackWidthInches = 18.5;
    private static final double kDriveGearRatio =
            6.746; // (50 / 14) * (17 / 27) * (45 / 15); // SDS L2
    private static final double kSteerGearRatio = 21.428; // (50.0 / 14.0) * (60.0 / 10.0); // SDS
    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;
    private static final double kCouplingGearRatio = 0; // SDS MK4

    // Simulation Config
    private static double kSteerInertia = 0.00001;
    private static double kDriveInertia = 0.001;

    // Wheel Positions
    private static final double kFrontLeftXPosInches = wheelBaseInches / 2.0;
    private static final double kFrontLeftYPosInches = trackWidthInches / 2.0;
    private static final double kFrontRightXPosInches = wheelBaseInches / 2.0;
    private static final double kFrontRightYPosInches = -trackWidthInches / 2.0;
    private static final double kBackLeftXPosInches = -wheelBaseInches / 2.0;
    private static final double kBackLeftYPosInches = trackWidthInches / 2.0;
    private static final double kBackRightXPosInches = -wheelBaseInches / 2.0;
    private static final double kBackRightYPosInches = -trackWidthInches / 2.0;

    public static class SlotGains extends Slot0Configs {
        public SlotGains(double kP, double kI, double kD, double kV, double kS) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kV = kV;
            this.kS = kS;
        }
    }

    public static final ModuleConfigFactory ModuleCreator =
            new ModuleConfigFactory()
                    .withDriveMotorGearRatio(kDriveGearRatio)
                    .withSteerMotorGearRatio(kSteerGearRatio)
                    .withCouplingGearRatio(kCouplingGearRatio)
                    .withWheelRadius(kWheelRadiusInches)
                    .withSlipCurrent(slipCurrent)
                    .withForwardTorqueCurrentLimit(peakForwardTorqueCurrent)
                    .withReverseTorqueCurrentLimit(PeakReverseTorqueCurrent)
                    .withSteerMotorGains(steerGains)
                    .withDriveMotorGains(driveGains)
                    .withSpeedAt12VoltsMps(speedAt12VoltsMps)
                    .withSteerInertia(kSteerInertia)
                    .withDriveInertia(kDriveInertia)
                    .withFeedbackSource(steerFeedbackType)
                    .withSteerMotorInverted(kSteerMotorReversed);

    public static final ModuleConfig FrontLeft =
            ModuleCreator.createModuleConfig(
                    kFrontLeftSteerMotorId,
                    kFrontLeftDriveMotorId,
                    kFrontLeftEncoderId,
                    kFrontLeftCANcoderOffset,
                    Units.inchesToMeters(kFrontLeftXPosInches),
                    Units.inchesToMeters(kFrontLeftYPosInches),
                    kInvertLeftSide);

    public static final ModuleConfig FrontRight =
            ModuleCreator.createModuleConfig(
                    kFrontRightSteerMotorId,
                    kFrontRightDriveMotorId,
                    kFrontRightEncoderId,
                    kFrontRightCANncoderOffset,
                    Units.inchesToMeters(kFrontRightXPosInches),
                    Units.inchesToMeters(kFrontRightYPosInches),
                    kInvertRightSide);

    public static final ModuleConfig BackLeft =
            ModuleCreator.createModuleConfig(
                    kBackLeftSteerMotorId,
                    kBackLeftDriveMotorId,
                    kBackLeftEncoderId,
                    kBackLeftCANcoderOffset,
                    Units.inchesToMeters(kBackLeftXPosInches),
                    Units.inchesToMeters(kBackLeftYPosInches),
                    kInvertLeftSide);

    public static final ModuleConfig BackRight =
            ModuleCreator.createModuleConfig(
                    kBackRightSteerMotorId,
                    kBackRightDriveMotorId,
                    kBackRightEncoderId,
                    kBackRightCANcoderOffset,
                    Units.inchesToMeters(kBackRightXPosInches),
                    Units.inchesToMeters(kBackRightYPosInches),
                    kInvertRightSide);

    public static final ModuleConfig[] ModuleConfigs = {FrontLeft, FrontRight, BackLeft, BackRight};

    public static final SwerveConfig DrivetrainConstants =
            new SwerveConfig()
                    .withPigeon2Id(kPigeonId)
                    .withCANbusName(kCANbusName)
                    .withSupportsPro(supportsPro)
                    .withModules(ModuleConfigs)
                    .withRotationGains(
                            kPRotationController, kIRotationController, kDRotationController);
}
