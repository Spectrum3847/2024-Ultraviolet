package frc.spectrumLib.swerve.config;

import com.ctre.phoenix6.configs.Slot0Configs;
import frc.spectrumLib.swerve.config.ModuleConfig.SwerveModuleSteerFeedbackType;

public class ModuleConfigFactory {
    /** Gear ratio between drive motor and wheel */
    public double DriveMotorGearRatio = 0;
    /** Gear ratio between steer motor and CANcoder An example ratio for the SDS Mk4: 12.8 */
    public double SteerMotorGearRatio = 0;
    /** Coupled gear ratio between the CANcoder and the drive motor */
    public double CouplingGearRatio = 0;
    /** Wheel radius of the driving wheel in inches */
    public double WheelRadius = 0;
    /** The maximum amount of current the drive motors can apply without slippage */
    public double SlipCurrent = 0;

    public double SupplyCurrentLimit = 0;

    public double SupplyCurrentThreshold = 0;

    /** The maximum amount of forward torque current the drive motors can apply */
    public double PeakForwardTorqueCurrent = 0;
    /** The maximum amount of reverse torque current the drive motors can apply */
    public double PeakReverseTorqueCurrent = 0;

    /** The steer motor gains */
    public Slot0Configs SteerMotorGains = new Slot0Configs();
    /** The drive motor gains */
    public Slot0Configs DriveMotorGains = new Slot0Configs();

    /** True if the steering motor is reversed from the CANcoder */
    public boolean SteerMotorInverted = false;

    /** The theoretical free speed of the robot if all drive motors were applied 12 V */
    public double SpeedAt12VoltsMps = 0;

    /** Sim-specific constants * */
    /** Azimuthal inertia in kilogram meters squared */
    public double SteerInertia = 0.001;
    /** Drive inertia in kilogram meters squared */
    public double DriveInertia = 0.001;

    public SwerveModuleSteerFeedbackType FeedbackSource =
            SwerveModuleSteerFeedbackType.RemoteCANcoder;

    public ModuleConfigFactory withDriveMotorGearRatio(double driveMotorGearRatio) {
        this.DriveMotorGearRatio = driveMotorGearRatio;
        return this;
    }

    public ModuleConfigFactory withSteerMotorGearRatio(double steerMotorGearRatio) {
        this.SteerMotorGearRatio = steerMotorGearRatio;
        return this;
    }

    public ModuleConfigFactory withCouplingGearRatio(double couplingGearRatio) {
        this.CouplingGearRatio = couplingGearRatio;
        return this;
    }

    public ModuleConfigFactory withWheelRadius(double wheelRadius) {
        this.WheelRadius = wheelRadius;
        return this;
    }

    public ModuleConfigFactory withSlipCurrent(double slipCurrent) {
        this.SlipCurrent = slipCurrent;
        return this;
    }

    public ModuleConfigFactory withSteerMotorGains(Slot0Configs steerMotorGains) {
        this.SteerMotorGains = steerMotorGains;
        return this;
    }

    public ModuleConfigFactory withDriveMotorGains(Slot0Configs driveMotorGains) {
        this.DriveMotorGains = driveMotorGains;
        return this;
    }

    public ModuleConfigFactory withSteerMotorInverted(boolean steerMotorInverted) {
        this.SteerMotorInverted = steerMotorInverted;
        return this;
    }

    public ModuleConfigFactory withSpeedAt12VoltsMps(double speedAt12VoltsMps) {
        this.SpeedAt12VoltsMps = speedAt12VoltsMps;
        return this;
    }

    public ModuleConfigFactory withSteerInertia(double steerInertia) {
        this.SteerInertia = steerInertia;
        return this;
    }

    public ModuleConfigFactory withDriveInertia(double driveInertia) {
        this.DriveInertia = driveInertia;
        return this;
    }

    public ModuleConfigFactory withFeedbackSource(SwerveModuleSteerFeedbackType feedbackSource) {
        this.FeedbackSource = feedbackSource;
        return this;
    }

    public ModuleConfigFactory withForwardTorqueCurrentLimit(double currentLimit) {
        this.PeakForwardTorqueCurrent = currentLimit;
        return this;
    }

    public ModuleConfigFactory withReverseTorqueCurrentLimit(double currentLimit) {
        this.PeakReverseTorqueCurrent = currentLimit;
        return this;
    }

    public ModuleConfig createModuleConfig(
            int steerId,
            int driveId,
            int cancoderId,
            double cancoderOffset,
            double locationX,
            double locationY,
            boolean driveMotorReversed) {
        return new ModuleConfig()
                .withSteerMotorId(steerId)
                .withDriveMotorId(driveId)
                .withCANcoderId(cancoderId)
                .withCANcoderOffset(cancoderOffset)
                .withLocationX(locationX)
                .withLocationY(locationY)
                .withDriveMotorGearRatio(DriveMotorGearRatio)
                .withSteerMotorGearRatio(SteerMotorGearRatio)
                .withCouplingGearRatio(CouplingGearRatio)
                .withWheelRadius(WheelRadius)
                .withSlipCurrent(SlipCurrent)
                .withSupplyCurrent(SupplyCurrentLimit, SupplyCurrentThreshold)
                .withForwardTorqueCurrentLimit(PeakForwardTorqueCurrent)
                .withReverseTorqueCurrentLimit(PeakReverseTorqueCurrent)
                .withSteerMotorGains(SteerMotorGains)
                .withDriveMotorGains(DriveMotorGains)
                .withSteerMotorInverted(SteerMotorInverted)
                .withDriveMotorInverted(driveMotorReversed)
                .withSpeedAt12VoltsMps(SpeedAt12VoltsMps)
                .withSimulationSteerInertia(SteerInertia)
                .withSimulationDriveInertia(DriveInertia)
                .withFeedbackSource(FeedbackSource);
    }
}
