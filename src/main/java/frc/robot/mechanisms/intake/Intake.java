package frc.robot.mechanisms.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.mechanism.TalonFXFactory;
import frc.spectrumLib.util.Conversions;
import org.littletonrobotics.junction.AutoLogOutput;

public class Intake extends Mechanism {
    public class IntakeConfig extends Config {

        /* Revolutions per min Intake Output */
        public double maxSpeed = 5000; //TODO: configure
        public double intake = 4000; //TODO: configure
        public double eject = -3000; //TODO: configure

        /* Percentage Intake Output */
        public double slowIntakePercentage = 0.06; //TODO: configure

        /* Intake config values */
        public double currentLimit = 12;
        public double threshold = 20;
        public double velocityKp = 0.156152;
        public double velocityKv = 0.12;
        public double velocityKs = 0.24;

        public IntakeConfig() {
            super("Intake", 60, "3847");
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(12 / 30); //TODO: configure
            configSupplyCurrentLimit(currentLimit, threshold, true);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive(); //TODO: configure
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
     * Runs the intake at a specified percentage of its maximum output.
     *
     * @param percent The percentage of the maximum output to run the intake at.
     */
    public Command runPercentage(double percent) {
        return run(() -> setPercentOutput(percent)).withName("Intake.runPercentage");
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

    @Override
    protected Config setConfig() {
        config = new IntakeConfig();
        return config;
    }
}
