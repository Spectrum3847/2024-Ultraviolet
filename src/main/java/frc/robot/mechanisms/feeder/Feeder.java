package frc.robot.mechanisms.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.spectrumLib.lasercan.LaserCanUtil;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.mechanism.TalonFXFactory;
import frc.spectrumLib.util.Conversions;
import org.littletonrobotics.junction.AutoLogOutput;

public class Feeder extends Mechanism {
    public class FeederConfig extends Config {

        /* Revolutions per min Feeder Output */
        public double maxSpeed = 5000; // TODO: configure
        public double feed = 4000; // TODO: configure
        public double testFeed = 250;
        public double intake = 250;
        public double eject = -3000; // TODO: configure
        public double launchEject = 500;
        public double feedToAmp = -3000;
        public double feedToLauncher = 900;
        public double slowFeed = 900; // 780
        public double slowEject = -780; // -780
        public double fastFeed = 1000;

        /* Percentage Feeder Output */
        public double slowFeederPercentage = 0.15; // TODO: configure

        public double testForwardPercent = 1;
        public double testBackPercent = -0.5;

        /* Feeder config values */
        public double currentLimit = 12;
        public double threshold = 20;
        public double velocityKp = 0.156152;
        public double velocityKv = 0.12;
        public double velocityKs = 0.24;

        public FeederConfig() {
            super("Feeder", 40, "3847");
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(12 / 30); // TODO: configure
            configSupplyCurrentLimit(currentLimit, threshold, false);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive(); // TODO: configure
            configMotionMagic(51, 205, 0);
        }
    }

    public FeederConfig config;
    public LaserCanUtil lasercan;

    public Feeder(boolean attached) {
        super(attached);
        if (attached) {
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);
        }

        lasercan = new LaserCanUtil(0);
    }

    @AutoLogOutput(key = "LaserCan/Measurement")
    public int getLaserCanDistance() {
        return lasercan.getDistance();
    }

    public boolean hasNote() {
        return getLaserCanDistance() < 300;
    }

    public boolean midNote() {
        return Math.abs(getLaserCanDistance()) - 10 <= 0;
    }

    public boolean endNote() {
        return getLaserCanDistance() > 250;
    }

    @Override
    public void periodic() {
        // System.out.println("MEASUREMENT: " + lasercan.getDistance());
    }

    /* Control methods: see method in lambda for more information */

    /**
     * Runs the feeder at a given velocity
     *
     * @param velocity in revolutions per minute
     */
    public Command runVelocity(double velocity) {
        return run(() -> setVelocity(Conversions.RPMtoRPS(velocity)))
                .withName("Feeder.runVelocity");
    }

    /**
     * Runs the feeder at a specified percentage of its maximum output.
     *
     * @param percent The percentage of the maximum output to run the feeder at.
     */
    public Command runPercentage(double percent) {
        return run(() -> setPercentOutput(percent)).withName("Feeder.runPercentage");
    }

    /**
     * Stops the feeder motor.
     *
     * @return
     */
    public Command runStop() {
        return run(() -> stop()).withName("Feeder.stop");
    }

    /**
     * Temporarily sets the feeder to coast mode. The configuration is applied when the command is
     * started and reverted when the command is ended.
     */
    public Command coastMode() {
        return startEnd(() -> setBrakeMode(false), () -> setBrakeMode(true))
                .ignoringDisable(true)
                .withName("Feeder.coastMode");
    }

    /* Logging */

    /** Returns the velocity of the motor in rotations per second */
    @AutoLogOutput(key = "Feeder/Motor Velocity (rotations per second)")
    public double getMotorVelocity() {
        if (attached) {
            return motor.getVelocity().getValueAsDouble();
        }
        return 0;
    }

    @Override
    protected Config setConfig() {
        config = new FeederConfig();
        return config;
    }
}
