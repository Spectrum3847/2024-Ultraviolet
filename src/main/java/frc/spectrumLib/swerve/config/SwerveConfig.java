package frc.spectrumLib.swerve.config;

public class SwerveConfig {
    /** CAN ID of the Pigeon2 on the drivetrain */
    public int Pigeon2Id = 0;
    /** Name of CANivore the swerve drive is on */
    public String CANbusName = "rio";

    /** If using Pro, specify this as true to make use of all the Pro features */
    public boolean SupportsPro = false;

    public ModuleConfig[] modules = new ModuleConfig[0];

    /*Rotation Controller*/
    public double kPRotationController = 0.0;
    public double kIRotationController = 0.0;
    public double kDRotationController = 0.0;

    /*Profiling Configs*/
    public double maxVelocity = 0;
    public double maxAccel = maxVelocity * 1.5; // take 1/2 sec to get to max speed.
    public double maxAngularVelocity = Math.PI * 2;
    public double maxAngularAcceleration = Math.pow(maxAngularVelocity, 2);

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

    public SwerveConfig withModules(ModuleConfig[] modules) {
        this.modules = modules;
        return this;
    }

    public SwerveConfig withRotationGains(double kP, double kI, double kD) {
        this.kPRotationController = kP;
        this.kIRotationController = kI;
        this.kDRotationController = kD;
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
}
