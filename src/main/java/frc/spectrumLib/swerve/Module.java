package frc.spectrumLib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.spectrumLib.swerve.config.ModuleConfig;

/**
 * Swerve Module class that encapsulates a swerve module powered by CTR Electronics devices.
 *
 * <p>This class handles the hardware devices and configures them for swerve module operation using
 * the Phoenix 6 API.
 *
 * <p>This class constructs hardware devices internally, so the user only specifies the constants
 * (IDs, PID gains, gear ratios, etc). Getters for these hardware devices are available.
 */
public class Module {
    private TalonFX m_driveMotor;
    private TalonFX m_steerMotor;
    private CANcoder m_cancoder;

    private StatusSignal<Double> m_drivePosition;
    private StatusSignal<Double> m_driveVelocity;
    private StatusSignal<Double> m_steerPosition;
    private StatusSignal<Double> m_steerVelocity;
    private BaseStatusSignal[] m_signals;
    private double m_driveRotationsPerMeter = 0;
    private double m_couplingRatioDriveRotorToCANcoder;

    private MotionMagicVoltage m_angleSetter = new MotionMagicVoltage(0);
    private VelocityTorqueCurrentFOC m_velocityTorqueSetter = new VelocityTorqueCurrentFOC(0);
    private VelocityVoltage m_velocityVoltageSetter = new VelocityVoltage(0);
    private VoltageOut m_voltageOpenLoopSetter = new VoltageOut(0);

    private SwerveModulePosition m_internalState = new SwerveModulePosition();
    private double m_speedAt12VoltsMps = 0;
    private boolean m_supportsPro = false;

    /**
     * Construct a SwerveModule with the specified constants.
     *
     * @param config Constants used to construct the module
     * @param canbusName The name of the CAN bus this module is on
     * @param supportsPro True if the devices are licensed to use Pro features
     */
    public Module(ModuleConfig config, String canbusName, boolean supportsPro) {
        m_driveMotor = new TalonFX(config.DriveMotorId, canbusName);
        m_steerMotor = new TalonFX(config.SteerMotorId, canbusName);
        m_cancoder = new CANcoder(config.AngleEncoderId, canbusName);

        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();

        talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        talonConfigs.Slot0 = config.DriveMotorGains;
        talonConfigs.TorqueCurrent.PeakForwardTorqueCurrent = config.SlipCurrent;
        talonConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -config.SlipCurrent;
        talonConfigs.CurrentLimits.StatorCurrentLimit = config.SlipCurrent;
        talonConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonConfigs.CurrentLimits.SupplyCurrentLimit = config.SupplyCurrentLimit;
        talonConfigs.CurrentLimits.SupplyCurrentThreshold = config.SupplyCurrentThreshold;
        talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        talonConfigs.MotorOutput.Inverted =
                config.DriveMotorInverted
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        Timer.delay(0.1);
        StatusCode response = m_driveMotor.getConfigurator().apply(talonConfigs);
        if (!response.isOK()) {
            System.out.println(
                    "Talon ID "
                            + config.DriveMotorId
                            + " failed config with error "
                            + response.toString());
        }

        /* Undo changes for torqueCurrent */
        talonConfigs.TorqueCurrent = new TorqueCurrentConfigs();
        /* And to current limits */
        talonConfigs.CurrentLimits = new CurrentLimitsConfigs();

        talonConfigs.Slot0 = config.SteerMotorGains;
        // Modify configuration to use remote CANcoder fused
        talonConfigs.Feedback.FeedbackRemoteSensorID = config.AngleEncoderId;
        switch (config.FeedbackSource) {
            case RemoteCANcoder:
                talonConfigs.Feedback.FeedbackSensorSource =
                        FeedbackSensorSourceValue.RemoteCANcoder;
                break;
            case FusedCANcoder:
                talonConfigs.Feedback.FeedbackSensorSource =
                        FeedbackSensorSourceValue.FusedCANcoder;
                break;
            case SyncCANcoder:
                talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
                break;
        }
        talonConfigs.Feedback.RotorToSensorRatio = config.SteerMotorGearRatio;

        talonConfigs.MotionMagic.MotionMagicAcceleration = config.MotionMagicAcceleration;
        talonConfigs.MotionMagic.MotionMagicCruiseVelocity = config.MotionMagicCruiseVelocity;

        talonConfigs.ClosedLoopGeneral.ContinuousWrap =
                true; // Enable continuous wrap for swerve modules

        talonConfigs.MotorOutput.Inverted =
                config.SteerMotorInverted
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;

        Timer.delay(0.1);
        response = m_steerMotor.getConfigurator().apply(talonConfigs);
        if (!response.isOK()) {
            System.out.println(
                    "Talon ID "
                            + config.DriveMotorId
                            + " failed config with error "
                            + response.toString());
        }

        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = config.CANcoderOffset;
        Timer.delay(0.1);
        response = m_cancoder.getConfigurator().apply(cancoderConfigs);
        if (!response.isOK()) {
            System.out.println(
                    "CANcoder ID "
                            + config.DriveMotorId
                            + " failed config with error "
                            + response.toString());
        }

        m_drivePosition = m_driveMotor.getPosition().clone();
        m_driveVelocity = m_driveMotor.getVelocity().clone();
        m_steerPosition = m_steerMotor.getPosition().clone();
        m_steerVelocity = m_steerMotor.getVelocity().clone();

        m_signals = new BaseStatusSignal[4];
        m_signals[0] = m_drivePosition;
        m_signals[1] = m_driveVelocity;
        m_signals[2] = m_steerPosition;
        m_signals[3] = m_steerVelocity;

        /* Calculate the ratio of drive motor rotation to meter on ground */
        double rotationsPerWheelRotation = config.DriveMotorGearRatio;
        double metersPerWheelRotation = 2 * Math.PI * Units.inchesToMeters(config.WheelRadius);
        m_driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;
        m_couplingRatioDriveRotorToCANcoder = config.CouplingGearRatio;

        /* Make control requests synchronous */
        m_velocityTorqueSetter.UpdateFreqHz = 0;
        m_velocityVoltageSetter.UpdateFreqHz = 0;
        m_voltageOpenLoopSetter.UpdateFreqHz = 0;
        m_angleSetter.UpdateFreqHz = 0;

        /* Get the expected speed when applying 12 volts */
        m_speedAt12VoltsMps = config.SpeedAt12VoltsMps;

        /* If this supports pro, save it */
        m_supportsPro = supportsPro;
    }

    /**
     * Gets the state of this module and passes it back as a SwerveModulePosition object with
     * latency compensated values.
     *
     * @param refresh True if the signals should be refreshed
     * @return SwerveModulePosition containing this module's state.
     */
    public SwerveModulePosition getPosition(boolean refresh) {
        if (refresh) {
            /* Refresh all signals */
            m_drivePosition.refresh();
            m_driveVelocity.refresh();
            m_steerPosition.refresh();
            m_steerVelocity.refresh();
        }

        /* Now latency-compensate our signals */
        double drive_rot =
                BaseStatusSignal.getLatencyCompensatedValue(m_drivePosition, m_driveVelocity);
        double angle_rot =
                BaseStatusSignal.getLatencyCompensatedValue(m_steerPosition, m_steerVelocity);

        /*
         * Back out the drive rotations based on angle rotations due to coupling between
         * azimuth and steer
         */
        drive_rot -= angle_rot * m_couplingRatioDriveRotorToCANcoder;

        /* And push them into a SwerveModuleState object to return */
        m_internalState.distanceMeters = drive_rot / m_driveRotationsPerMeter;
        /* Angle is already in terms of steer rotations */
        m_internalState.angle = Rotation2d.fromRotations(angle_rot);

        return m_internalState;
    }

    /**
     * Applies the desired SwerveModuleState to this module.
     *
     * @param state Speed and direction the module should target
     * @param isOpenLoop True if this should use open-loop control.
     */
    public void apply(SwerveModuleState state, boolean isOpenLoop) {
        SwerveModuleState optimized = SwerveModuleState.optimize(state, m_internalState.angle);

        /* Back out the expected shimmy the drive motor will see */
        /* Find the angular rate to determine what to back out */
        double azimuthTurnRps = m_steerVelocity.getValue();
        /* Azimuth turn rate multiplied by coupling ratio provides back-out rps */
        double driveRateBackOut = azimuthTurnRps * m_couplingRatioDriveRotorToCANcoder;

        double angleToSetDeg = optimized.angle.getRotations();
        m_steerMotor.setControl(m_angleSetter.withPosition(angleToSetDeg));
        double velocityToSet =
                (optimized.speedMetersPerSecond * m_driveRotationsPerMeter) - driveRateBackOut;

        /* From FRC 900's whitepaper, we add a cosine compensator to the applied drive velocity */
        /* To reduce the "skew" that occurs when changing direction */
        double steerMotorError = angleToSetDeg - m_steerPosition.getValue();
        // If error is close to 0 rotations, we're already there, so apply full power
        // If the error is close to 0.25 rotations, then we're 90 degrees, so movement doesn't help
        // us at all
        // We take the absolute value of this to make sure we don't invert our drive, even though we
        // shouldn't ever target over 90 degrees anyway
        double cosineScalar = Math.abs(Math.cos(Units.rotationsToRadians(steerMotorError)));
        velocityToSet *= cosineScalar;

        if (isOpenLoop) {
            /* Open loop ignores the driveRotationsPerMeter since it only cares about the open loop at the mechanism */
            /* But we do care about the backout due to coupling, so we keep it in */
            velocityToSet /= m_driveRotationsPerMeter;
            /* Open loop always uses voltage setter */
            m_driveMotor.setControl(
                    m_voltageOpenLoopSetter.withOutput(velocityToSet / m_speedAt12VoltsMps * 12.0));
        } else {
            /* If we support pro, use the torque request */
            if (m_supportsPro) {
                m_driveMotor.setControl(m_velocityTorqueSetter.withVelocity(velocityToSet));
            } else {
                m_driveMotor.setControl(m_velocityVoltageSetter.withVelocity(velocityToSet));
            }
        }
    }

    /**
     * Gets the last cached swerve module position. This differs from {@link getPosition} in that it
     * will not perform any latency compensation or refresh the signals.
     *
     * @return Last cached SwerveModulePosition
     */
    public SwerveModulePosition getCachedPosition() {
        return m_internalState;
    }

    /**
     * Get the current state of the module.
     *
     * <p>This is typically used for telemetry, as the SwerveModulePosition is used for odometry.
     *
     * @return Current state of the module
     */
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
                m_driveVelocity.getValue() / m_driveRotationsPerMeter,
                Rotation2d.fromRotations(m_steerPosition.getValue()));
    }

    /**
     * Gets the position/velocity signals of the drive and steer
     *
     * @return Array of BaseStatusSignals for this module in the following order: 0 - Drive Position
     *     1 - Drive Velocity 2 - Steer Position 3 - Steer Velocity
     */
    BaseStatusSignal[] getSignals() {
        return m_signals;
    }

    /** Resets this module's drive motor position to 0 rotations. */
    public void resetPosition() {
        /* Only touch drive pos, not steer */
        m_driveMotor.setPosition(0);
    }

    /**
     * Gets this module's Drive Motor TalonFX reference.
     *
     * <p>This should be used only to access signals and change configurations that the swerve
     * drivetrain does not configure itself.
     *
     * @return This module's Drive Motor reference
     */
    public TalonFX getDriveMotor() {
        return m_driveMotor;
    }

    /**
     * Gets this module's Steer Motor TalonFX reference.
     *
     * <p>This should be used only to access signals and change configurations that the swerve
     * drivetrain does not configure itself.
     *
     * @return This module's Steer Motor reference
     */
    public TalonFX getSteerMotor() {
        return m_steerMotor;
    }

    /**
     * Gets this module's CANcoder reference.
     *
     * <p>This should be used only to access signals and change configurations that the swerve
     * drivetrain does not configure itself.
     *
     * @return This module's CANcoder reference
     */
    public CANcoder getCANcoder() {
        return m_cancoder;
    }

    public void setModuleNeutralMode(NeutralModeValue mode) {
        setDriveNeutralMode(mode);
        setSteerNeutralMode(mode);
    }

    public void setDriveNeutralMode(NeutralModeValue mode) {
        MotorOutputConfigs driveConfig = new MotorOutputConfigs();
        m_driveMotor.getConfigurator().refresh(driveConfig);
        if (driveConfig.NeutralMode != mode) {
            driveConfig.NeutralMode = mode;
            m_driveMotor.getConfigurator().apply(driveConfig);
        }
    }

    public void setSteerNeutralMode(NeutralModeValue mode) {
        MotorOutputConfigs steerConfig = new MotorOutputConfigs();
        m_steerMotor.getConfigurator().refresh(steerConfig);
        if (steerConfig.NeutralMode != mode) {
            steerConfig.NeutralMode = mode;
            m_driveMotor.getConfigurator().apply(steerConfig);
        }
    }
}
