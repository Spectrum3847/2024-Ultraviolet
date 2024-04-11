package frc.spectrumLib.swerve.config;

import com.ctre.phoenix6.configs.Slot0Configs;

public class ModuleConfig {
    public enum SwerveModuleSteerFeedbackType {
        RemoteCANcoder,
        FusedCANcoder,
        SyncCANcoder,
    }

    /** CAN ID of the drive motor */
    public int DriveMotorId = 0;
    /** CAN ID of the steer motor */
    public int SteerMotorId = 0;
    /** CAN ID of the CANcoder used for azimuth or RIO AnalogInput ID of the Analog Sensor used */
    public int AngleEncoderId = 0;
    /** Offset of the CANcoder in degrees */
    public double CANcoderOffset = 0;
    /** Gear ratio between drive motor and wheel */
    public double DriveMotorGearRatio = 0;
    /** Gear ratio between steer motor and CANcoder An example ratio for the SDS Mk4: 12.8 */
    public double SteerMotorGearRatio = 0;
    /** Coupled gear ratio between the CANcoder and the drive motor */
    public double CouplingGearRatio = 0;
    /** Wheel radius of the driving wheel in inches */
    public double WheelRadius = 0;
    /**
     * The location of this module's wheels relative to the physical center of the robot in meters
     * along the X axis of the robot
     */
    public double LocationX = 0;
    /**
     * The location of this module's wheels relative to the physical center of the robot in meters
     * along the Y axis of the robot
     */
    public double LocationY = 0;

    /** The steer motor gains */
    public Slot0Configs SteerMotorGains = new Slot0Configs();
    /** The drive motor gains */
    public Slot0Configs DriveMotorGains = new Slot0Configs();

    /** The maximum amount of current the drive motors can apply without slippage */
    public double SlipCurrent = 400;

    public double SupplyCurrentLimit = 400;

    public double SupplyCurrentThreshold = 400;

    /** The maximum amount of forward torque current the drive motors can apply */
    public double PeakForwardTorqueCurrent = 300;
    /** The maximum amount of reverse torque current the drive motors can apply */
    public double PeakReverseTorqueCurrent = 300;

    /** True if the steering motor is reversed from the CANcoder */
    public boolean SteerMotorInverted = false;
    /** True if the driving motor is reversed */
    public boolean DriveMotorInverted = false;

    /** Sim-specific constants * */
    /** Azimuthal inertia in kilogram meters squared */
    public double SteerInertia = 0.001;
    /** Drive inertia in kilogram meters squared */
    public double DriveInertia = 0.001;

    /**
     * When using open-loop drive control, this specifies the speed the robot travels at when driven
     * with 12 volts in meters per second. This is used to approximate the output for a desired
     * velocity. If using closed loop control, this value is ignored
     */
    public double SpeedAt12VoltsMps = 0;

    /**
     * Choose how the feedback sensors should be configured
     *
     * <p>If the robot does not support Pro, then this should remain as RemoteCANcoder. Otherwise,
     * users have the option to use either FusedCANcoder or SyncCANcoder depending on if there is
     * risk that the CANcoder can fail in a way to provide "good" data.
     */
    public SwerveModuleSteerFeedbackType FeedbackSource =
            SwerveModuleSteerFeedbackType.RemoteCANcoder;

    public double MotionMagicAcceleration = 100.0;
    public double MotionMagicCruiseVelocity = 10;

    public ModuleConfig withDriveMotorId(int id) {
        this.DriveMotorId = id;
        return this;
    }

    public ModuleConfig withSteerMotorId(int id) {
        this.SteerMotorId = id;
        return this;
    }

    public ModuleConfig withCANcoderId(int id) {
        this.AngleEncoderId = id;
        return this;
    }

    public ModuleConfig withCANcoderOffset(double offset) {
        this.CANcoderOffset = offset;
        return this;
    }

    public ModuleConfig withDriveMotorGearRatio(double ratio) {
        this.DriveMotorGearRatio = ratio;
        return this;
    }

    public ModuleConfig withSteerMotorGearRatio(double ratio) {
        this.SteerMotorGearRatio = ratio;
        return this;
    }

    public ModuleConfig withCouplingGearRatio(double ratio) {
        this.CouplingGearRatio = ratio;
        return this;
    }

    public ModuleConfig withWheelRadius(double radius) {
        this.WheelRadius = radius;
        return this;
    }

    public ModuleConfig withLocationX(double locationXMeters) {
        this.LocationX = locationXMeters;
        return this;
    }

    public ModuleConfig withLocationY(double locationYMeters) {
        this.LocationY = locationYMeters;
        return this;
    }

    public ModuleConfig withSteerMotorGains(Slot0Configs gains) {
        this.SteerMotorGains = gains;
        return this;
    }

    public ModuleConfig withDriveMotorGains(Slot0Configs gains) {
        this.DriveMotorGains = gains;
        return this;
    }

    public ModuleConfig withSlipCurrent(double slipCurrent) {
        this.SlipCurrent = slipCurrent;
        return this;
    }

    public ModuleConfig withSupplyCurrent(
            double supplyCurrentLimit, double supplyCurrentThreshold) {
        this.SupplyCurrentLimit = supplyCurrentLimit;
        this.SupplyCurrentThreshold = supplyCurrentThreshold;
        return this;
    }

    public ModuleConfig withForwardTorqueCurrentLimit(double currentLimit) {
        this.PeakForwardTorqueCurrent = currentLimit;
        return this;
    }

    public ModuleConfig withReverseTorqueCurrentLimit(double currentLimit) {
        this.PeakReverseTorqueCurrent = currentLimit;
        return this;
    }

    public ModuleConfig withSteerMotorInverted(boolean SteerMotorInverted) {
        this.SteerMotorInverted = SteerMotorInverted;
        return this;
    }

    public ModuleConfig withDriveMotorInverted(boolean DriveMotorInverted) {
        this.DriveMotorInverted = DriveMotorInverted;
        return this;
    }

    public ModuleConfig withSpeedAt12VoltsMps(double speedAt12VoltsMps) {
        this.SpeedAt12VoltsMps = speedAt12VoltsMps;
        return this;
    }

    public ModuleConfig withSimulationSteerInertia(double steerInertia) {
        this.SteerInertia = steerInertia;
        return this;
    }

    public ModuleConfig withSimulationDriveInertia(double driveInertia) {
        this.DriveInertia = driveInertia;
        return this;
    }

    public ModuleConfig withFeedbackSource(SwerveModuleSteerFeedbackType source) {
        this.FeedbackSource = source;
        return this;
    }

    public ModuleConfig withMotionMagicAcceleration(double acceleration) {
        this.MotionMagicAcceleration = acceleration;
        return this;
    }

    public ModuleConfig withMotionMagicCruiseVelocity(double cruiseVelocity) {
        this.MotionMagicCruiseVelocity = cruiseVelocity;
        return this;
    }
}
