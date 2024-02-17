package frc.spectrumLib.mechanism;

public class RobotMechConfig {
    public BaseMechConfig AMPTRAP_CONFIG = new BaseMechConfig();
    public BaseMechConfig CLIMBER_CONFIG = new BaseMechConfig();
    public BaseMechConfig ELEVATOR_CONFIG = new BaseMechConfig();
    public BaseMechConfig FEEDER_CONFIG = new BaseMechConfig();
    public BaseMechConfig INTAKE_CONFIG = new BaseMechConfig();
    public BaseMechConfig LEFTLAUNCHER_CONFIG = new BaseMechConfig();
    public BaseMechConfig RIGHTLAUNCHER_CONFIG = new BaseMechConfig();
    public BaseMechConfig PIVOT_CONFIG = new BaseMechConfig();

    public RobotMechConfig(
            BaseMechConfig ampTrapConfig,
            BaseMechConfig climberConfig,
            BaseMechConfig elevatorConfig,
            BaseMechConfig feederConfig,
            BaseMechConfig intakeConfig,
            BaseMechConfig leftLauncherConfig,
            BaseMechConfig rightLauncherConfig,
            BaseMechConfig pivotConfig) {
        AMPTRAP_CONFIG = ampTrapConfig;
        CLIMBER_CONFIG = climberConfig;
        ELEVATOR_CONFIG = elevatorConfig;
        FEEDER_CONFIG = feederConfig;
        INTAKE_CONFIG = intakeConfig;
        LEFTLAUNCHER_CONFIG = leftLauncherConfig;
        RIGHTLAUNCHER_CONFIG = rightLauncherConfig;
        PIVOT_CONFIG = pivotConfig;
    }
}
