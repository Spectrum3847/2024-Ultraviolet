package frc.spectrumLib.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.spectrumLib.swerve.config.ModuleConfig;
import frc.spectrumLib.swerve.config.SwerveConfig;

/**
 * Extremely simplified swerve drive simulation class.
 *
 * <p>This class assumes that the swerve drive is perfect, meaning that there is no scrub and the
 * wheels do not slip.
 *
 * <p>In addition, it assumes the inertia of the robot is governed only by the inertia of the steer
 * module and the individual drive wheels. Robot-wide inertia is not accounted for, and neither is
 * translational vs rotational inertia of the robot.
 *
 * <p>These assumptions provide a simplified example that can demonstrate the behavior of a swerve
 * drive in simulation. Users are encouraged to expand this model for their own use.
 */
public class SimDrivetrain {
    public class SimSwerveModule {
        /* Reference to motor simulation for the steer motor */
        public final DCMotorSim SteerMotor;
        /* Reference to motor simulation for drive motor */
        public final DCMotorSim DriveMotor;
        /* Refernece to steer gearing for updating CANcoder */
        public final double SteerGearing;
        /* Refernece to steer gearing for updating CANcoder */
        public final double DriveGearing;

        public SimSwerveModule(
                double steerGearing,
                double steerInertia,
                double driveGearing,
                double driveInertia) {
            SteerMotor = new DCMotorSim(DCMotor.getFalcon500(1), steerGearing, steerInertia);
            DriveMotor = new DCMotorSim(DCMotor.getFalcon500(1), driveGearing, driveInertia);
            SteerGearing = steerGearing;
            DriveGearing = driveGearing;
        }
    }

    public final Pigeon2SimState PigeonSim;
    protected final SimSwerveModule[] m_modules;
    protected final SwerveModulePosition[] m_lastPositions;
    private final int ModuleCount;
    public final SwerveDriveKinematics Kinem;
    public Rotation2d LastAngle = new Rotation2d();

    public SimDrivetrain(
            Translation2d[] wheelLocations,
            Pigeon2 pigeon,
            SwerveConfig swerveConfig,
            ModuleConfig... moduleConfigs) {
        PigeonSim = pigeon.getSimState();
        ModuleCount = moduleConfigs.length;
        m_modules = new SimSwerveModule[ModuleCount];
        m_lastPositions = new SwerveModulePosition[ModuleCount];
        for (int i = 0; i < ModuleCount; ++i) {
            m_modules[i] =
                    new SimSwerveModule(
                            moduleConfigs[i].SteerMotorGearRatio,
                            moduleConfigs[i].SteerInertia,
                            moduleConfigs[i].DriveMotorGearRatio,
                            moduleConfigs[i].DriveInertia);
            m_lastPositions[i] = new SwerveModulePosition(0, new Rotation2d());
        }

        Kinem = new SwerveDriveKinematics(wheelLocations);
    }

    /**
     * Update this simulation for the time duration.
     *
     * <p>This performs a simulation update on all the simulated devices
     *
     * @param dtSeconds The time delta between this update and the previous update
     * @param supplyVoltage The voltage as seen at the motor controllers
     * @param modulesToApply What modules to apply the update to
     */
    public void update(double dtSeconds, double supplyVoltage, Module... modulesToApply) {
        if (m_modules.length != ModuleCount) return;

        SwerveModulePosition[] positions = new SwerveModulePosition[ModuleCount];
        /* Update our sim devices */
        for (int i = 0; i < ModuleCount; ++i) {
            TalonFXSimState steerMotor = modulesToApply[i].getSteerMotor().getSimState();
            TalonFXSimState driveMotor = modulesToApply[i].getDriveMotor().getSimState();
            CANcoderSimState cancoder = modulesToApply[i].getCANcoder().getSimState();

            m_modules[i].SteerMotor.setInputVoltage(steerMotor.getMotorVoltage());
            m_modules[i].DriveMotor.setInputVoltage(driveMotor.getMotorVoltage());

            m_modules[i].SteerMotor.update(dtSeconds);
            m_modules[i].DriveMotor.update(dtSeconds);

            steerMotor.setRawRotorPosition(
                    m_modules[i].SteerMotor.getAngularPositionRotations()
                            * m_modules[i].SteerGearing);
            steerMotor.setRotorVelocity(
                    m_modules[i].SteerMotor.getAngularVelocityRPM()
                            / 60.0
                            * m_modules[i].SteerGearing);
            steerMotor.setSupplyVoltage(supplyVoltage);

            /* CANcoders see the mechanism, so don't account for the steer gearing */
            cancoder.setRawPosition(m_modules[i].SteerMotor.getAngularPositionRotations());
            cancoder.setVelocity(m_modules[i].SteerMotor.getAngularVelocityRPM() / 60.0);
            cancoder.setSupplyVoltage(supplyVoltage);

            driveMotor.setRawRotorPosition(
                    m_modules[i].DriveMotor.getAngularPositionRotations()
                            * m_modules[i].DriveGearing);
            driveMotor.setRotorVelocity(
                    m_modules[i].DriveMotor.getAngularVelocityRPM()
                            / 60.0
                            * m_modules[i].DriveGearing);
            driveMotor.setSupplyVoltage(supplyVoltage);

            SwerveModulePosition currentPos = modulesToApply[i].getCachedPosition();
            positions[i] =
                    new SwerveModulePosition(
                            currentPos.distanceMeters - m_lastPositions[i].distanceMeters,
                            currentPos.angle);
            m_lastPositions[i].distanceMeters = currentPos.distanceMeters;
        }

        Twist2d change = Kinem.toTwist2d(positions);
        LastAngle = LastAngle.plus(Rotation2d.fromRadians(change.dtheta));
        PigeonSim.setRawYaw(LastAngle.getDegrees());
    }
}
