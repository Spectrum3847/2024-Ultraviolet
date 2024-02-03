package frc.robot.mechanisms.amptrap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.mechanism.TalonFXFactory;
import frc.spectrumLib.util.Conversions;
import org.littletonrobotics.junction.AutoLogOutput;

public class AmpTrap extends Mechanism {
    public class AmpTrapConfig extends Config {

        /* Revolutions per min AmpTrap Output */
        public double maxSpeed = 5000; // TODO: configure
        public double intake = 750; // TODO: configure
        public double score = -3000; // TODO: configure
        public double launchEject = 500;
        public double eject = 3000;

        /* Percentage AmpTrap Output */
        public double slowIntakePercentage = 0.06; // TODO: configure

        public double testForwardPercent = 1;
        public double testBackPercent = -0.50;

        /* AmpTrap config values */
        public double currentLimit = 12;
        public double threshold = 20;
        public double velocityKp = 0.156152;
        public double velocityKv = 0.12;
        public double velocityKs = 0.24;

        public AmpTrapConfig() {
            super("AmpTrap", 51, "3847");
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(12 / 30); // TODO: configure
            configSupplyCurrentLimit(currentLimit, threshold, false);
            configNeutralBrakeMode(true);
            configClockwise_Positive(); // TODO: configure
            configMotionMagic(51, 205, 0);
        }
    }

    public AmpTrapConfig config;

    public AmpTrap(boolean attached) {
        super(attached);
        if (attached) {
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);
        }
    }

    @Override
    public void periodic() {}

    /* Control methods: see method in lambda for more information */

    /**
     * Runs the AmpTrap at a given velocity
     *
     * @param velocity in revolutions per minute
     */
    public Command runVelocity(double velocity) {
        return run(() -> setVelocity(Conversions.RPMtoRPS(velocity)))
                .withName("AmpTrap.runVelocity");
    }

    /**
     * Runs the AmpTrap at a specified percentage of its maximum output.
     *
     * @param percent The percentage of the maximum output to run the AmpTrap at.
     */
    public Command runPercentage(double percent) {
        return run(() -> setPercentOutput(percent)).withName("AmpTrap.runPercentage");
    }

    /**
     * Temporarily sets the amptrap to coast mode. The configuration is applied when the command is
     * started and reverted when the command is ended.
     */
    public Command coastMode() {
        return startEnd(() -> setBrakeMode(false), () -> setBrakeMode(true))
                .ignoringDisable(true)
                .withName("AmpTrap.coastMode");
    }

    /**
     * Stops the AmpTrap motor.
     *
     * @return
     */
    public Command runStop() {
        return run(() -> stop()).withName("AmpTrap.stop");
    }

    /* Logging */

    /** Returns the velocity of the motor in rotations per second */
    @AutoLogOutput(key = "AmpTrap/Motor Velocity (rotations per second)")
    public double getMotorVelocity() {
        if (attached) {
            return motor.getVelocity().getValueAsDouble();
        }
        return 0;
    }

    @Override
    protected Config setConfig() {
        config = new AmpTrapConfig();
        return config;
    }
}
