package frc.robot.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotTelemetry;
import frc.robot.swerve.configs.MUSICDISC2023;
import frc.robot.swerve.configs.NOTEBLOCK2023;
import frc.robot.swerve.configs.ULTRAVIOLET2024;
import frc.spectrumLib.swerve.Drivetrain;
import frc.spectrumLib.swerve.Drivetrain.DriveState;
import frc.spectrumLib.swerve.Module;
import frc.spectrumLib.swerve.Request;
import frc.spectrumLib.swerve.config.SwerveConfig;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Swerve implements Subsystem {
    public final SwerveConfig config;
    private final Drivetrain drivetrain;
    private final RotationController rotationController;
    private double OdometryUpdateFrequency = 250;
    private double targetHeading = 0;
    private ReadWriteLock m_stateLock = new ReentrantReadWriteLock();
    private SwerveModuleState[] Setpoints = new SwerveModuleState[] {};

    public Swerve() {
        RobotTelemetry.print("Swerve Subsystem Starting: ");

        // Choose the correct swerve configuration
        switch (Robot.config.getRobotType()) {
            case ULTRAVIOLET:
                config = ULTRAVIOLET2024.config;
                break;
            case NOTEBLOCK:
                config = NOTEBLOCK2023.config;
                break;
            case MUSICDISC:
                config = MUSICDISC2023.config;
                break;
            case SIM: // runs in simulation
                OdometryUpdateFrequency = 50;
                config = NOTEBLOCK2023.config;
                break;
            default:
                config = NOTEBLOCK2023.config;
                break;
        }
        drivetrain = new Drivetrain(config, OdometryUpdateFrequency);

        rotationController = new RotationController(this);
        RobotTelemetry.print("Swerve Subsystem Initialized: ");
    }

    @Override
    public void periodic() {
        // Log measured states
        Logger.recordOutput("SwerveStates/Measured", drivetrain.getState().ModuleStates);

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
        } else {
            // Log setpoint states
            Logger.recordOutput("SwerveStates/Setpoints", readSetpoints());
            ;
        }

        // Log Odometry Pose
        Logger.recordOutput("Odometry/Robot", getPose());
    }

    @Override
    public void simulationPeriodic() {
        drivetrain.updateSimState(0.02, 12);
    }

    // Returns a commmand that applies the given request to the drivetrain
    public Command applyRequest(Supplier<Request> requestSupplier) {
        return run(() -> setControlMode(requestSupplier.get()));
    }

    // Use this to control the swerve drive, set motors, etc.
    public void setControlMode(Request mode) {
        drivetrain.setControl(mode);
    }

    public DriveState getState() {
        return drivetrain.getState();
    }

    public Pose2d getPose() {
        return getState().Pose;
    }

    public void resetPose(Pose2d pose) {
        drivetrain.seedFieldRelative(pose);
    }

    public void reorient(double angle) {
        drivetrain.seedFieldRelative(angle);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return drivetrain.getChassisSpeeds();
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public void resetRotationController() {
        rotationController.reset();
    }

    public Module[] getModules() {
        return drivetrain.getModules();
    }

    public double calculateRotationController(DoubleSupplier targetRadians) {
        return rotationController.calculate(targetRadians.getAsDouble());
    }

    public void setTargetHeading(double targetHeading) {
        this.targetHeading = targetHeading;
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    /**
     * Takes the current orientation of the robot plus an angle offset and makes it X forward for
     * field-relative maneuvers.
     */
    public void seedFieldRelative(double offsetDegrees) {
        drivetrain.seedFieldRelative(offsetDegrees);
    }

    /** This will zero the entire odometry, and place the robot at 0,0 */
    public void zeroOdoemtry() {
        drivetrain.tareEverything();
    }

    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        drivetrain.addVisionMeasurement(
                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        drivetrain.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
        drivetrain.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
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
        drivetrain.registerTelemetry(telemetryFunction);
    }

    /**
     * Applies the specified control request to this swerve drivetrain.
     *
     * @param request Request to apply
     */
    public void writeSetpoints(SwerveModuleState[] setpoints) {
        try {
            m_stateLock.writeLock().lock();

            this.Setpoints = setpoints;
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    public SwerveModuleState[] readSetpoints() {
        try {
            m_stateLock.readLock().lock();
            return Setpoints;
        } finally {
            m_stateLock.readLock().unlock();
        }
    }
}
