package frc.spectrumLib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import frc.robot.RobotTelemetry;
import frc.spectrumLib.swerve.Request.ControlRequestParameters;
import frc.spectrumLib.swerve.config.ModuleConfig;
import frc.spectrumLib.swerve.config.SwerveConfig;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Consumer;

/**
 * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API.
 *
 * <p>This class handles the kinematics, configuration, and odometry of a swerve drive utilizing CTR
 * Electronics devices. We recommend that users use the Swerve Mechanism Generator in Tuner X to
 * create a template project that demonstrates how to use this class.
 *
 * <p>This class will construct the hardware devices internally, so the user only specifies the
 * constants (IDs, PID gains, gear ratios, etc). Getters for these hardware devices are available.
 *
 * <p>If using the generator, the order in which modules are constructed is Front Left, Front Right,
 * Back Left, Back Right. This means if you need the Back Left module, call {@code getModule(2);} to
 * get the 3rd index (0-indexed) module, corresponding to the Back Left module.
 */
public class Drivetrain {
    protected final int ModuleCount;
    protected final double UpdateFrequency;
    protected final Module[] Modules;

    protected Pigeon2 m_pigeon2;
    protected SwerveDriveKinematics m_kinematics;
    protected SwerveDrivePoseEstimator m_odometry;
    protected SwerveModulePosition[] m_modulePositions;
    protected Translation2d[] m_moduleLocations;
    protected OdometryThread m_odometryThread;
    protected Rotation2d m_fieldRelativeOffset;
    protected StatusSignal<Double> m_yawGetter;
    protected StatusSignal<Double> m_angularZGetter;

    protected Request m_requestToApply = new Request.Idle();
    protected ControlRequestParameters m_requestParameters = new ControlRequestParameters();

    protected ReadWriteLock m_stateLock = new ReentrantReadWriteLock();

    protected final SimDrivetrain m_simDrive;
    protected final boolean IsOnCANFD;

    /**
     * Plain-Old-Data class holding the state of the swerve drivetrain. This encapsulates most data
     * that is relevant for telemetry or decision-making from the Swerve Drive.
     */
    public class DriveState {
        public int SuccessfulDaqs;
        public int FailedDaqs;
        public Pose2d Pose = new Pose2d();
        public SwerveModuleState[] ModuleStates = new SwerveModuleState[] {};
        public double OdometryPeriod;
    }

    protected Consumer<DriveState> m_telemetryFunction = null;
    protected DriveState m_cachedState = new DriveState();

    /* Perform swerve module updates in a separate thread to minimize latency */
    public class OdometryThread extends Thread {
        private final int START_THREAD_PRIORITY =
                1; // Testing shows 1 (minimum realtime) is sufficient for tighter
        // odometry loops.
        // If the odometry period is far away from the desired frequency,
        // increasing this may help

        private BaseStatusSignal[] m_allSignals;
        public int SuccessfulDaqs = 0;
        public int FailedDaqs = 0;
        MedianFilter peakRemover = new MedianFilter(3);
        LinearFilter lowPass = LinearFilter.movingAverage(50);
        double lastTime = 0;
        double currentTime = 0;
        double averageLoopTime = 0;

        int lastThreadPriority = START_THREAD_PRIORITY;
        int threadPriorityToSet = START_THREAD_PRIORITY;

        public OdometryThread() {
            super();
            // 4 signals for each module + 2 for Pigeon2
            m_allSignals = new BaseStatusSignal[(ModuleCount * 4) + 2];
            for (int i = 0; i < ModuleCount; ++i) {
                BaseStatusSignal[] signals = Modules[i].getSignals();
                m_allSignals[(i * 4) + 0] = signals[0];
                m_allSignals[(i * 4) + 1] = signals[1];
                m_allSignals[(i * 4) + 2] = signals[2];
                m_allSignals[(i * 4) + 3] = signals[3];
            }
            m_allSignals[m_allSignals.length - 2] = m_yawGetter;
            m_allSignals[m_allSignals.length - 1] = m_angularZGetter;
        }

        @Override
        public void run() {
            /* Make sure all signals update at around 250hz */
            for (BaseStatusSignal sig : m_allSignals) {
                sig.setUpdateFrequency(UpdateFrequency);
            }
            Threads.setCurrentThreadPriority(true, START_THREAD_PRIORITY);

            /* Run as fast as possible, our signals will control the timing */
            while (true) {
                /* Synchronously wait for all signals in drivetrain */
                /* Wait up to twice the period of the update frequency */
                StatusCode status;
                if (IsOnCANFD) {
                    status = BaseStatusSignal.waitForAll(2.0 / UpdateFrequency, m_allSignals);
                } else {
                    try {
                        /* Wait for the signals to update */
                        Thread.sleep((long) ((1.0 / UpdateFrequency) * 1000.0));
                    } catch (InterruptedException ex) {
                    }
                    status = BaseStatusSignal.refreshAll(m_allSignals);
                }
                m_stateLock.writeLock().lock();

                lastTime = currentTime;
                currentTime = Utils.getCurrentTimeSeconds();
                /*
                 * We don't care about the peaks, as they correspond to GC events, and we want
                 * the period generally low passed
                 */
                averageLoopTime = lowPass.calculate(peakRemover.calculate(currentTime - lastTime));

                /* Get status of first element */
                if (status.isOK()) {
                    SuccessfulDaqs++;
                } else {
                    FailedDaqs++;
                }

                /* Now update odometry */
                /* Keep track of the change in azimuth rotations */
                for (int i = 0; i < ModuleCount; ++i) {
                    m_modulePositions[i] = Modules[i].getPosition(false);
                }
                // Assume Pigeon2 is flat-and-level so latency compensation can be performed
                double yawDegrees =
                        BaseStatusSignal.getLatencyCompensatedValue(m_yawGetter, m_angularZGetter);

                /* Keep track of previous and current pose to account for the carpet vector */
                m_odometry.update(Rotation2d.fromDegrees(yawDegrees), m_modulePositions);

                /* And now that we've got the new odometry, update the controls */
                m_requestParameters.currentPose =
                        m_odometry
                                .getEstimatedPosition()
                                .relativeTo(new Pose2d(0, 0, m_fieldRelativeOffset));
                m_requestParameters.kinematics = m_kinematics;
                m_requestParameters.swervePositions = m_moduleLocations;
                m_requestParameters.timestamp = currentTime;
                m_requestParameters.updatePeriod = 1.0 / UpdateFrequency;

                m_requestToApply.apply(m_requestParameters, Modules);

                /* Update our cached state with the newly updated data */
                m_cachedState.FailedDaqs = FailedDaqs;
                m_cachedState.SuccessfulDaqs = SuccessfulDaqs;
                m_cachedState.ModuleStates = new SwerveModuleState[Modules.length];
                for (int i = 0; i < Modules.length; ++i) {
                    m_cachedState.ModuleStates[i] = Modules[i].getCurrentState();
                }
                m_cachedState.Pose = m_odometry.getEstimatedPosition();
                m_cachedState.OdometryPeriod = averageLoopTime;

                if (m_telemetryFunction != null) {
                    /* Log our state */
                    m_telemetryFunction.accept(m_cachedState);
                }

                m_stateLock.writeLock().unlock();
                /**
                 * This is inherently synchronous, since lastThreadPriority is only written here and
                 * threadPriorityToSet is only read here
                 */
                if (threadPriorityToSet != lastThreadPriority) {
                    Threads.setCurrentThreadPriority(true, threadPriorityToSet);
                    lastThreadPriority = threadPriorityToSet;
                }
            }
        }

        public boolean odometryIsValid() {
            return SuccessfulDaqs > 2; // Wait at least 3 daqs before saying the odometry is valid
        }

        /**
         * Sets the DAQ thread priority to a real time priority under the specified priority level
         *
         * @param priority Priority level to set the DAQ thread to. This is a value between 0 and
         *     99, with 99 indicating higher priority and 0 indicating lower priority.
         */
        public void setThreadPriority(int priority) {
            threadPriorityToSet = priority;
        }
    }

    protected boolean checkIsOnCanFD(String canbusName) {
        return CANBus.isNetworkFD(canbusName);
    }

    /**
     * Constructs a CTRSwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so user should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param swerveConfig Drivetrain-wide constants for the swerve drive
     * @param modules Constants for each specific module
     */
    public Drivetrain(SwerveConfig swerveConfig) {
        this(swerveConfig, 250);
    }

    /**
     * Constructs a CTRSwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so user should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param swerveConfig Drivetrain-wide constants for the swerve drive
     * @param OdometryUpdateFrequency The frequency to run the odometry loop. If unspecified, this
     *     is 250 hz.
     * @param modules Constants for each specific module
     */
    public Drivetrain(SwerveConfig swerveConfig, double OdometryUpdateFrequency) {
        ModuleConfig[] moduleConfigs = swerveConfig.modules;
        UpdateFrequency = OdometryUpdateFrequency;
        ModuleCount = moduleConfigs.length;

        IsOnCANFD = checkIsOnCanFD(swerveConfig.CANbusName);

        m_pigeon2 = new Pigeon2(swerveConfig.Pigeon2Id, swerveConfig.CANbusName);
        m_yawGetter = m_pigeon2.getYaw().clone();
        m_angularZGetter = m_pigeon2.getAngularVelocityZWorld().clone();

        Modules = new Module[ModuleCount];
        m_modulePositions = new SwerveModulePosition[ModuleCount];
        m_moduleLocations = new Translation2d[ModuleCount];

        int iteration = 0;
        for (ModuleConfig module : moduleConfigs) {
            Modules[iteration] =
                    new Module(module, swerveConfig.CANbusName, swerveConfig.SupportsPro);
            m_moduleLocations[iteration] = new Translation2d(module.LocationX, module.LocationY);
            m_modulePositions[iteration] = Modules[iteration].getPosition(true);

            iteration++;
        }
        m_kinematics = new SwerveDriveKinematics(m_moduleLocations);
        m_odometry =
                new SwerveDrivePoseEstimator(
                        m_kinematics, new Rotation2d(), m_modulePositions, new Pose2d());

        m_fieldRelativeOffset = new Rotation2d();

        m_simDrive = new SimDrivetrain(m_moduleLocations, m_pigeon2, swerveConfig, moduleConfigs);

        m_odometryThread = new OdometryThread();
        RobotTelemetry.print("Starting Odometry Thread: ");
        m_odometryThread.start();
    }

    /**
     * Gets a reference to the data acquisition thread.
     *
     * @return DAQ thread
     */
    public OdometryThread getDaqThread() {
        return m_odometryThread;
    }

    /**
     * Applies the specified control request to this swerve drivetrain.
     *
     * @param request Request to apply
     */
    public void setControl(Request request) {
        try {
            m_stateLock.writeLock().lock();

            m_requestToApply = request;
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Zero's this swerve drive's odometry entirely.
     *
     * <p>This will zero the entire odometry, and place the robot at 0,0
     */
    public void tareEverything() {
        try {
            m_stateLock.writeLock().lock();

            for (int i = 0; i < ModuleCount; ++i) {
                Modules[i].resetPosition();
                m_modulePositions[i] = Modules[i].getPosition(true);
            }
            m_odometry.resetPosition(m_pigeon2.getRotation2d(), m_modulePositions, new Pose2d());
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Takes the current orientation of the robot and makes it X forward for field-relative
     * maneuvers.
     */
    public void seedFieldRelative() {
        try {
            m_stateLock.writeLock().lock();

            m_fieldRelativeOffset = getState().Pose.getRotation();
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Blue alliance sees forward as 0 degrees (toward red alliance wall) Red alliance sees forward
     * as 180 degrees (toward blue alliance wall)
     */
    public void setDriverPerspective(Rotation2d rotation) {
        try {
            m_stateLock.writeLock().lock();

            m_fieldRelativeOffset = rotation;
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /* Temporary Method */
    public void setBlueAllianceRotation() {
        setDriverPerspective(Rotation2d.fromDegrees(0));
    }

    public void setRedAllianceRotation() {
        setDriverPerspective(Rotation2d.fromDegrees(180));
    }

    /**
     * Takes the current orientation of the robot plus an angle offset and makes it X forward for
     * field-relative maneuvers.
     */
    public void seedFieldRelative(double offsetDegrees) {
        try {
            m_stateLock.writeLock().lock();

            // m_fieldRelativeOffset = 0
            // getState().Pose.getRotation().plus(Rotation2d.fromDegrees(offsetDegrees));
            Rotation2d heading =
                    m_pigeon2.getRotation2d().plus(Rotation2d.fromDegrees(offsetDegrees));
            m_odometry.resetPosition(heading, m_modulePositions, new Pose2d());

            if (DriverStation.getRawAllianceStation() == AllianceStationID.Red1
                    || DriverStation.getRawAllianceStation() == AllianceStationID.Red2
                    || DriverStation.getRawAllianceStation() == AllianceStationID.Red3) {
                this.setRedAllianceRotation();
            } else {
                this.setBlueAllianceRotation();
            }

        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Only reset pose angle, not pose location
     *
     * @param offsetDegrees
     */
    public void reorient(double degrees) {
        try {
            m_stateLock.writeLock().lock();

            m_odometry.resetPosition(
                    m_pigeon2.getRotation2d(),
                    m_modulePositions,
                    new Pose2d(
                            m_odometry.getEstimatedPosition().getX(),
                            m_odometry.getEstimatedPosition().getY(),
                            Rotation2d.fromDegrees(degrees)));
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Takes the specified location and makes it the current pose for field-relative maneuvers
     *
     * @param location Pose to make the current pose at.
     */
    public void seedFieldRelative(Pose2d location) {
        try {
            m_stateLock.writeLock().lock();

            m_odometry.resetPosition(
                    Rotation2d.fromDegrees(m_yawGetter.getValue()), m_modulePositions, location);
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Check if the odometry is currently valid
     *
     * @return True if odometry is valid
     */
    public boolean odometryIsValid() {
        try {
            m_stateLock.readLock().lock();

            return m_odometryThread.odometryIsValid();
        } finally {
            m_stateLock.readLock().unlock();
        }
    }

    /**
     * Get a reference to the module at the specified index. The index corresponds to the module
     * described in the constructor
     *
     * @param index Which module to get
     * @return Reference to SwerveModule
     */
    public Module getModule(int index) {
        if (index >= Modules.length) return null;
        return Modules[index];
    }

    public void setSwerveNeutralMode(NeutralModeValue mode) {
        for (Module module : Modules) {
            module.setModuleNeutralMode(mode);
        }
    }

    /**
     * Gets the current state of the swerve drivetrain.
     *
     * @return Current state of the drivetrain
     */
    public DriveState getState() {
        try {
            m_stateLock.readLock().lock();

            return m_cachedState;
        } finally {
            m_stateLock.readLock().unlock();
        }
    }

    /**
     * Get the current module states from the cached state
     *
     * @return
     */
    public SwerveModuleState[] getModuleStates() {
        return getState().ModuleStates;
    }

    /**
     * Get the current chassis speeds from the cached state
     *
     * @return
     */
    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * <p>This method can be called as infrequently as you want, as long as you are calling {@link
     * SwerveDrivePoseEstimator#update} every loop.
     *
     * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
     * recommend only adding vision measurements that are already within one meter or so of the
     * current pose estimate.
     *
     * <p>Note that the vision measurement standard deviations passed into this method will continue
     * to apply to future measurements until a subsequent call to {@link
     * SwerveDrivePoseEstimator#setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
     *     don't use your own time source by calling {@link
     *     SwerveDrivePoseEstimator#updateWithTime(double,Rotation2d,SwerveModulePosition[])}, then
     *     you must use a timestamp with an epoch since FPGA startup (i.e., the epoch of this
     *     timestamp is the same epoch as {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}).
     *     This means that you should use {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()} as
     *     your time source in this case.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x
     *     position in meters, y position in meters, and heading in radians). Increase these numbers
     *     to trust the vision pose measurement less.
     */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        try {
            m_stateLock.writeLock().lock();
            m_odometry.addVisionMeasurement(
                    visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * <p>This method can be called as infrequently as you want, as long as you are calling {@link
     * SwerveDrivePoseEstimator#update} every loop.
     *
     * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
     * recommend only adding vision measurements that are already within one meter or so of the
     * current pose estimate.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
     *     don't use your own time source by calling {@link
     *     SwerveDrivePoseEstimator#updateWithTime(double,Rotation2d,SwerveModulePosition[])} then
     *     you must use a timestamp with an epoch since FPGA startup (i.e., the epoch of this
     *     timestamp is the same epoch as {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}.)
     *     This means that you should use {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()} as
     *     your time source or sync the epochs.
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        try {
            m_stateLock.writeLock().lock();
            m_odometry.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Sets the pose estimator's trust of global measurements. This might be used to change trust in
     * vision measurements after the autonomous period, or to change trust as distance to a vision
     * target increases.
     *
     * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase
     *     these numbers to trust global measurements from vision less. This matrix is in the form
     *     [x, y, theta]ᵀ, with units in meters and radians.
     */
    public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
        try {
            m_stateLock.writeLock().lock();
            m_odometry.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Updates all the simulation state variables for this drivetrain class. User provides the
     * update variables for the simulation.
     *
     * @param dtSeconds time since last update call
     * @param supplyVoltage voltage as seen at the motor controllers
     */
    public void updateSimState(double dtSeconds, double supplyVoltage) {
        m_simDrive.update(dtSeconds, supplyVoltage, Modules);
    }

    /**
     * Register the specified lambda to be executed whenever our SwerveDriveState function is
     * updated in our odometry thread.
     *
     * <p>It is imperative that this function is cheap, as it will be executed along with the
     * odometry call, and if this takes a long time, it may negatively impact the odometry of this
     * stack.
     *
     * <p>This can also be used for logging data if the function performs logging instead of
     * telemetry
     *
     * @param telemetryFunction Function to call for telemetry or logging
     */
    public void registerTelemetry(Consumer<DriveState> telemetryFunction) {
        m_telemetryFunction = telemetryFunction;
    }
}
