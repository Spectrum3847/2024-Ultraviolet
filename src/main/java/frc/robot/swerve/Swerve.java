package frc.robot.swerve;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.crescendo.Field;
import frc.robot.Robot;
import frc.robot.RobotTelemetry;
import frc.robot.swerve.configs.ALPHA2024;
import frc.robot.swerve.configs.MUSICDISC2023;
import frc.robot.swerve.configs.NOTEBLOCK2023;
import frc.robot.swerve.configs.PM2024;
import frc.robot.swerve.configs.ULTRAVIOLET2024;
import frc.robot.vision.Vision.VisionConfig;
import frc.spectrumLib.swerve.Drivetrain;
import frc.spectrumLib.swerve.Drivetrain.DriveState;
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
    private final AlignmentController xController;
    private final AlignmentController yController;
    private Field2d field = new Field2d();
    private double OdometryUpdateFrequency = 250;
    private double targetHeading = 0;
    private ReadWriteLock m_stateLock = new ReentrantReadWriteLock();

    private SwerveModuleState[] Setpoints = new SwerveModuleState[] {};

    public Swerve() {
        RobotTelemetry.print("Swerve Subsystem Starting: ");

        // Choose the correct swerve configuration
        switch (Robot.config.getRobotType()) {
            case PM:
                config = PM2024.config;
                break;
            case ULTRAVIOLET:
                config = ULTRAVIOLET2024.config;
                break;
            case ALPHA:
                config = ALPHA2024.config;
                break;
            case NOTEBLOCK:
                config = NOTEBLOCK2023.config;
                break;
            case MUSICDISC:
                config = MUSICDISC2023.config;
                break;
            case SIM: // runs in simulation
                OdometryUpdateFrequency = 50;
                config = ULTRAVIOLET2024.config;
                break;
            default:
                DriverStation.reportError(
                        "Could not match robot to swerve config; defaulting to PM2024 swerve config",
                        false);
                config = ULTRAVIOLET2024.config;
                break;
        }
        drivetrain = new Drivetrain(config, OdometryUpdateFrequency);

        rotationController = new RotationController(this);

        // Setup alignment controllers with 1/2 velocity and accel
        xController =
                new AlignmentController(this)
                        .withConstraints(config.maxVelocity / 2, config.maxAccel / 2);
        yController =
                new AlignmentController(this)
                        .withConstraints(config.maxVelocity / 2, config.maxAccel / 2);

        setVisionMeasurementStdDevs(VisionConfig.visionStdMatrix);

        SmartDashboard.putData("Odometry/Field", field);
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
        Logger.recordOutput("Odometry/RobotX", getPose().getX());
        Logger.recordOutput("Odometry/RobotY", getPose().getY());

        // Log Vision Pose
        if (Robot.isReal()) {
            // Logger.recordOutput(
            //         "Vision/Front/Pose", Robot.vision.speakerLL.getRawPose3d().toPose2d());
            // Logger.recordOutput(
            //         "Vision/Front/PoseX",
            // Robot.vision.speakerLL.getRawPose3d().toPose2d().getX());
            // Logger.recordOutput(
            //         "Vision/Front/PoseY",
            // Robot.vision.speakerLL.getRawPose3d().toPose2d().getY());
            // Logger.recordOutput(
            //         "Vision/Front/TagDistance",
            //         Robot.vision.speakerLL.getDistanceToTagFromCamera());

            // Logger.recordOutput("Vision/Rear/Pose",
            // Robot.vision.rearLL.getRawPose3d().toPose2d());
            // Logger.recordOutput(
            //         "Vision/Rear/PoseX", Robot.vision.rearLL.getRawPose3d().toPose2d().getX());
            // Logger.recordOutput("Vision/Rear/PoseY", Robot.vision.rearLL.getRawPose3d().getY());
            // Logger.recordOutput(
            //         "Vision/Rear/TagDistance", Robot.vision.rearLL.getDistanceToTagFromCamera());

            // Logger.recordOutput("Vision/Left/Pose",
            // Robot.vision.leftLL.getRawPose3d().toPose2d());
            // Logger.recordOutput(
            //         "Vision/Right/Pose", Robot.vision.rightLL.getRawPose3d().toPose2d());
        }

        // Update Field object for smartdashboard
        field.setRobotPose(getPose());
        field.getObject("Vision").setPose(Robot.vision.speakerLL.getRawPose3d().toPose2d());
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
        drivetrain.reorient(angle);
    }

    public void setBrakeMode() {
        drivetrain.setSwerveNeutralMode(NeutralModeValue.Brake);
    }

    public void setCoastMode() {
        drivetrain.setSwerveNeutralMode(NeutralModeValue.Coast);
    }

    public void reorientForward() {
        double angle = 0;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            angle = 180;
        }
        drivetrain.reorient(angle);
    }

    public void reorientLeft() {
        double angle = 90;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            angle = 270;
        }
        drivetrain.reorient(angle);
    }

    public void reorientRight() {
        double angle = 270;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            angle = 90;
        }
        drivetrain.reorient(angle);
    }

    public void reorientBack() {
        double angle = 180;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            angle = 0;
        }
        drivetrain.reorient(angle);
    }

    public void cardinalReorient() {
        double angle = getClosestCardinal();
        drivetrain.reorient(angle);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return drivetrain.getChassisSpeeds();
    }

    /**
     * Gets the robot's velocity.
     *
     * @param fieldRelative If the returned velocity should be field relative.
     */
    public ChassisSpeeds getVelocity(boolean fieldRelative) {
        if (fieldRelative) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(getRobotRelativeSpeeds(), getRotation());
        } else {
            return getRobotRelativeSpeeds();
        }
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public double getClosestCardinal() {
        double heading = getRotation().getRadians();
        if (heading > -Math.PI / 4 && heading <= Math.PI / 4) {
            return 0;
        } else if (heading > Math.PI / 4 && heading <= 3 * Math.PI / 4) {
            return 90;
        } else if (heading > 3 * Math.PI / 4 || heading <= -3 * Math.PI / 4) {
            return 180;
        } else {
            return 270;
        }
    }

    public void resetRotationController() {
        rotationController.reset();
    }

    public double calculateRotationController(DoubleSupplier targetRadians) {
        return rotationController.calculate(targetRadians.getAsDouble());
    }

    public boolean rotationControllerAtSetpoint() {
        return rotationController.atSetpoint();
    }

    public boolean rotationControllerAtFeedback() {
        return rotationController.atFeedbackSetpoint();
    }

    public void setTargetHeading(double targetHeading) {
        this.targetHeading = targetHeading;
    }

    public void resetAlignmentControllers() {
        Pose2d pose = getPose();
        xController.reset(pose.getX());
        yController.reset(pose.getY());
    }

    public void resetXController() {
        xController.reset(getPose().getX());
    }

    public double calculateXController(DoubleSupplier targetMeters) {
        double velocity = xController.calculate(getPose().getX(), targetMeters.getAsDouble());

        if (Field.isRed()) {
            return -velocity;
        }
        return velocity;
    }

    public void resetYController() {
        yController.reset(getPose().getY());
    }

    public double calculateYController(DoubleSupplier targetMeters) {
        double velocity = yController.calculate(getPose().getY(), targetMeters.getAsDouble());

        if (Field.isRed()) {
            return -velocity;
        }
        return velocity;
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    /*Temporary Method */
    public void setBlueAllianceRotation() {
        setDriverPerspective(Rotation2d.fromDegrees(0));
    }

    public void setRedAllianceRotation() {
        setDriverPerspective(Rotation2d.fromDegrees(180));
    }

    public void setDriverPerspective(Rotation2d perspective) {
        drivetrain.setDriverPerspective(perspective);
    }

    /**
     * Takes the current orientation of the robot plus an angle offset and makes it X forward for
     * field-relative maneuvers.
     */
    public void seedFieldRelative(double offsetDegrees) {
        drivetrain.seedFieldRelative(offsetDegrees);
    }

    public Pose2d convertPoseWithGyro(Pose2d pose) {
        return new Pose2d(pose.getX(), pose.getY(), getRotation());
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
