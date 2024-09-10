package frc.spectrumLib.swerve.config;

import frc.robot.mechanisms.pivot.Pivot.CANCoderFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
public class SwerveConfig {
    /** CAN ID of the Pigeon2 on the drivetrain */
    public int Pigeon2Id = 0;
    /** Name of CANivore the swerve drive is on */
    public String CANbusName = "rio";

    /** If using Pro, specify this as true to make use of all the Pro features */
    public boolean SupportsPro = false;

    public SwerveModuleConstants[] modules = new SwerveModuleConstants[0];

    /*Rotation Controller*/
    public double kPRotationController = 0.0;
    public double kIRotationController = 0.0;
    public double kDRotationController = 0.0;

    /*Alignment Controller */
    public double kPAlignmentController = 0.0;
    public double kIAlignmentController = 0.0;
    public double kDAlignmentController = 0.0;

    /*Profiling Configs*/
    public double maxVelocity = 0;
    public double maxAccel = maxVelocity * 1.5; // take 1/2 sec to get to max speed.
    public double maxAngularVelocity = Math.PI * 2;
    public double maxAngularAcceleration = Math.pow(maxAngularVelocity, 2);

    /* Deadbanding */
    public double deadband = 0; // fractional units 0 - 1
    public double rotationDeadband = 0; // fractionaln units 0 -1

    /* Pivot CANCoder config */
    public double pivotCANcoderOffset = 0; // flip sign
    public CANCoderFeedbackType pivotFeedbackSource = CANCoderFeedbackType.FusedCANcoder;

    public SwerveConfig withPigeon2Id(int id) {
        this.Pigeon2Id = id;
        return this;
    }

    public SwerveConfig withCANbusName(String name) {
        this.CANbusName = name;
        return this;
    }

    public SwerveConfig withSupportsPro(boolean supportsPro) {
        this.SupportsPro = supportsPro;
        return this;
    }

    public SwerveConfig withModules(SwerveModuleConstants[] modules) {
        this.modules = modules;
        return this;
    }

    public SwerveConfig withRotationGains(double kP, double kI, double kD) {
        this.kPRotationController = kP;
        this.kIRotationController = kI;
        this.kDRotationController = kD;
        return this;
    }

    public SwerveConfig withAlignmentGains(double kP, double kI, double kD) {
        this.kPAlignmentController = kP;
        this.kIAlignmentController = kI;
        this.kDAlignmentController = kD;
        return this;
    }

    public SwerveConfig withProfilingConfigs(
            double maxVelocity,
            double maxAccel,
            double maxAngularVelocity,
            double maxAngularAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAccel = maxAccel;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxAngularAcceleration = maxAngularAcceleration;
        return this;
    }

    public SwerveConfig withDeadbandConfig(double deadband, double rotationDeadband) {
        this.deadband = deadband;
        this.rotationDeadband = rotationDeadband;
        return this;
    }

    public SwerveConfig withPivotConfig(
            double pivotCANcoderOffset, CANCoderFeedbackType pivotFeedbackSource) {
        this.pivotCANcoderOffset = pivotCANcoderOffset;
        this.pivotFeedbackSource = pivotFeedbackSource;
        return this;
    }
}
