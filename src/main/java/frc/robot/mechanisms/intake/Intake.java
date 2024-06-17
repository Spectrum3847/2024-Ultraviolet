package frc.robot.mechanisms.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.mechanism.TalonFXFactory;
import frc.spectrumLib.util.Conversions;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Intake extends Mechanism {
    public class IntakeConfig extends Config {

        /* Revolutions per min Intake Output */
        public double maxSpeed = 5000; // TODO: configure
        public double intake = 2000; // 2000
        public double testVelocity = 3000;
        public double eject = -2000;

        /* Percentage Intake Output */
        public double ejectPercentage = -0.5; // TODO: configure
        public double slowIntakePercentage = 0.06; // TODO: configure
        public double testIntakePercentage = 0.7;

        /* Intake config values */
        public double currentLimit = 30;
        public double threshold = 40;
        public double torqueCurrentLimit = 120;
        public double velocityKp = 12; // 0.156152;
        public double velocityKv = 0.2; // 0.12;
        public double velocityKs = 14;

        public IntakeConfig() {
            super("Intake", 60, "3847");
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(12 / 30); // TODO: configure
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configSupplyCurrentLimit(currentLimit, threshold, true);
            configStatorCurrentLimit(80, true);
            configNeutralBrakeMode(true);
            configClockwise_Positive(); // TODO: configure
            configMotionMagic(51, 205, 0);
        }
    }

    public IntakeConfig config;

    public Intake(boolean attached) {
        super(attached);
        if (attached) {
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);
        }
    }

    @Override
    public void periodic() {}

    /* Control methods: see method in lambda for more information */

    /**
     * Runs the intake at a given velocity
     *
     * @param velocity in revolutions per minute
     */
    public Command runVelocity(double velocity) {
        return run(() -> setVelocity(Conversions.RPMtoRPS(velocity)))
                .withName("Intake.runVelocity");
    }

    /**
     * Run the left launcher at given velocity in TorqueCurrentFOC mode
     *
     * @param percent
     * @return
     */
    public Command runVelocityTorqueCurrentFOC(double velocity) {
        return run(() -> setVelocityTorqueCurrentFOC(Conversions.RPMtoRPS(velocity)))
                .withName("Intake.runVelocityFOC");
    }

    /**
     * Runs the intake at a specified percentage of its maximum output.
     *
     * @param percent The percentage of the maximum output to run the intake at.
     */
    public Command runPercentage(double percent) {
        return run(() -> setPercentOutput(percent)).withName("Intake.runPercentage");
    }

    public Command runManualOutput(DoubleSupplier percentSupplier) {
        return run(() -> setPercentOutput(percentSupplier.getAsDouble()))
                .withName("Intake.runManualOutput");
    }

    /**
     * Temporarily sets the intake to coast mode. The configuration is applied when the command is
     * started and reverted when the command is ended.
     */
    public Command coastMode() {
        return startEnd(() -> setBrakeMode(false), () -> setBrakeMode(true))
                .ignoringDisable(true)
                .withName("Intake.coastMode");
    }

    /**
     * Stops the intake motor.
     *
     * @return
     */
    public Command runStop() {
        return run(() -> stop()).withName("Intake.stop");
    }

    /* Logging */

    /** Returns the velocity of the motor in rotations per second */
    @AutoLogOutput(key = "Intake/Motor Velocity (rotations per second)")
    public double getMotorVelocity() {
        if (attached) {
            return motor.getVelocity().getValueAsDouble();
        }
        return 0;
    }

    /*
     * Returns the supply current of the motor in amps
     */
    public double getSupplyCurrent() {
        return motor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    protected Config setConfig() {
        config = new IntakeConfig();
        return config;
    }
}
