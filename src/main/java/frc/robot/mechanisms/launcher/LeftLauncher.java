package frc.robot.mechanisms.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.mechanism.TalonFXFactory;
import frc.spectrumLib.util.Conversions;
import org.littletonrobotics.junction.AutoLogOutput;

// TODO: this will get merged with right launcher into one class later
public class LeftLauncher extends Mechanism {
    public class LeftLauncherConfig extends Config {

        /* Revolutions per min LeftLauncher Output */
        public double maxSpeed = 5000; // TODO: configure
        public double launch = 4000; // TODO: configure
        public double testVelocity = 3000;

        /* Percentage LeftLauncher Output */
        public double slowLeftLauncherPercentage = 0.06; // TODO: configure
        public double testForwardPercent = -0.5;
        public double testBackPercent = 0.5;

        /* LeftLauncher config values */
        public double currentLimit = 12;
        public double threshold = 20;
        public double velocityKp = 0.156152;
        public double velocityKv = 0.12;
        public double velocityKs = 0.24;

        public LeftLauncherConfig() {
            super("LeftLauncher", 42, "3847");
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1 / 2); // TODO: configure
            configSupplyCurrentLimit(currentLimit, threshold, false);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive(); // TODO: configure
            configMotionMagic(51, 205, 0);
        }
    }

    public LeftLauncherConfig config;

    public LeftLauncher(boolean attached) {
        super(attached);
        if (attached) {
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);
        }
    }

    @Override
    public void periodic() {}

    /* Control methods: see method in lambda for more information */

    /**
     * Runs the left launcher at a given velocity
     *
     * @param velocity in revolutions per minute
     */
    public Command runVelocity(double velocity) {
        return run(() -> setVelocity(Conversions.RPMtoRPS(velocity)))
                .withName("LeftLauncher.runVelocity");
    }

    /**
     * Runs the left launcher at a specified percentage of its maximum output.
     *
     * @param percent The percentage of the maximum output to run the left launcher at.
     */
    public Command runPercentage(double percent) {
        return run(() -> setPercentOutput(percent)).withName("LeftLauncher.runPercentage");
    }

    /**
     * Stops the left launcher motor.
     *
     * @return
     */
    public Command runStop() {
        return run(() -> stop()).withName("LeftLauncher.stop");
    }

    /* Logging */

    /** Returns the velocity of the motor in rotations per second */
    @AutoLogOutput(key = "LeftLauncher/Motor Velocity (rotations per second)")
    public double getMotorVelocity() {
        if (attached) {
            return motor.getVelocity().getValueAsDouble();
        }
        return 0;
    }

    /** Returns the velocity of the motor in rotations per second */
    @AutoLogOutput(key = "LeftLauncher/Motor Velocity (revolutions per minute)")
    public double getMotorVelocityInRPM() {
        if (attached) {
            return Conversions.RPStoRPM(getMotorVelocity());
        }
        return 0;
    }

    @Override
    protected Config setConfig() {
        config = new LeftLauncherConfig();
        return config;
    }
}
