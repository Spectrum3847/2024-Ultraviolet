package frc.robot.swerve.configs;

import frc.spectrumLib.swerve.config.DefaultConfig;
import frc.spectrumLib.swerve.config.ModuleConfig;
import frc.spectrumLib.swerve.config.ModuleConfig.SwerveModuleSteerFeedbackType;
import frc.spectrumLib.swerve.config.SwerveConfig;

public class MUSICDISC2023 {
    // Angle Offsets
    private static final double kFrontLeftCANcoderOffset = 20;
    private static final double kFrontRightCANncoderOffset = 20;
    private static final double kBackLeftCANcoderOffset = 20;
    private static final double kBackRightCANcoderOffset = 20;

    // Device Setup
    private static final String kCANbusName = "rio";
    private static final boolean supportsPro = false;
    private static final SwerveModuleSteerFeedbackType steerFeedbackType =
            SwerveModuleSteerFeedbackType.RemoteCANcoder;

    public static final ModuleConfig FrontLeft =
            NOTEBLOCK2023.FrontLeft.withCANcoderOffset(kFrontLeftCANcoderOffset)
                    .withFeedbackSource(steerFeedbackType);
    public static final ModuleConfig FrontRight =
            NOTEBLOCK2023.FrontRight.withCANcoderOffset(kFrontRightCANncoderOffset)
                    .withFeedbackSource(steerFeedbackType);
    public static final ModuleConfig BackLeft =
            NOTEBLOCK2023.BackLeft.withCANcoderOffset(kBackLeftCANcoderOffset)
                    .withFeedbackSource(steerFeedbackType);
    public static final ModuleConfig BackRight =
            NOTEBLOCK2023.BackRight.withCANcoderOffset(kBackRightCANcoderOffset)
                    .withFeedbackSource(steerFeedbackType);

    public static final ModuleConfig[] ModuleConfigs = {FrontLeft, FrontRight, BackLeft, BackRight};

    public static final SwerveConfig config =
            DefaultConfig.DrivetrainConstants.withCANbusName(kCANbusName)
                    .withSupportsPro(supportsPro)
                    .withModules(ModuleConfigs);
}
