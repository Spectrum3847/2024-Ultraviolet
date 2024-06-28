package frc.robot.mechanisms.feeder;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConfig;
import frc.robot.RobotTelemetry;
import frc.spectrumLib.lasercan.LaserCanUtil;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.mechanism.TalonFXFactory;
import frc.spectrumLib.util.Conversions;

public class Feeder extends Mechanism {
    public class FeederConfig extends Config {

        /* Revolutions per min Feeder Output */
        public double maxSpeed = 5000;
        public double intake = 200;
        public double eject = -3000;
        public double score = 1000;
        public double slowFeed = 500;
        public double slowEject = -500;
        public double feedToAmp = -3500; // Needs to be greater than or equal to amp roller speed
        public double launchEject = 1000;
        public double autoFeed = 3000;
        public double ejectFromIntake = 3000;
        public double manualSource = -2000;

        /* Rotations config */
        public double addedFeedRotations = 4;

        /* Percentage Feeder Output */
        public double slowFeederPercentage = 0.15;

        /* Feeder config values */
        public double currentLimit = 30;
        public double torqueCurrentLimit = 100;
        public double threshold = 40;
        public double velocityKp = 0.156152;
        public double velocityKv = 0.12;
        public double velocityKs = 0.24;
        public double positionKp = 2;
        public double positionKv = 0.013;

        public FeederConfig() {
            super("Feeder", 40, RobotConfig.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0); // velocity
            configFeedForwardGains(velocityKs, velocityKv, 0, 0); // velocity
            configPIDGains(1, positionKp, 0, 0);
            configFeedForwardGains(1, 0, positionKv, 0, 0);
            configGearRatio(12 / 30);
            configSupplyCurrentLimit(currentLimit, threshold, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
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

    /* Lasercan */

    // @AutoLogOutput(key = "Feeder/LaserCan-Measurement")
    public int getLaserCanDistance() {
        return lasercan.getDistance();
    }

    // @AutoLogOutput(key = "Feeder/LaserCan-Valid")
    public boolean getLaserCanStatus() {
        return lasercan.validDistance();
    }

    /**
     * Fall back to using motor velocity if lasercan is not working/disconnected
     *
     * @return
     */
    public boolean intakedNote() {
        if (lasercan.validDistance()) {
            return lasercan.intakedNote();
        } else {
            RobotTelemetry.print("RESORTED TO FALLBACK INTAKE NOTE CHECK");
            return getMotorVelocity() > 0.01;
        }
    }

    public boolean noteIsClose() {
        if (lasercan.validDistance()) {
            return lasercan.closeNote();
        } else {
            return true; // assume that note is being stopped at feeder roller (close to lasercan)
            // and isn't being automatically fed up
        }
    }

    /* Feeder */

    @Override
    public void periodic() {}

    /* Control methods: see method in lambda for more information */

    /**
     * Runs the feeder a certain amount of revolutions forward from it's current position.
     *
     * @param revolutions position in revolutions
     */
    public Command runAddPosition(double revolutions) {
        return runOnce(this::tareMotor).andThen(run(() -> setMMPosition(revolutions, 1)));
    }

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

    /** Sets the motor to brake mode if it is in coast mode */
    public Command ensureBrakeMode() {
        return runOnce(
                        () -> {
                            setBrakeMode(true);
                        })
                .onlyIf(
                        () ->
                                attached
                                        && config.talonConfig.MotorOutput.NeutralMode
                                                == NeutralModeValue.Coast)
                .ignoringDisable(true);
    }

    /* Logging */

    /** Returns the velocity of the motor in rotations per second */
    // @AutoLogOutput(key = "Feeder/Motor Velocity (rotations per second)")
    public double getMotorVelocity() {
        if (attached) {
            return motor.getVelocity().getValueAsDouble();
        }
        return 0;
    }

    /** Intaked note status */
    // @AutoLogOutput(key = "Feeder/Note Intaked")
    public boolean noteIntaked() {
        if (attached) {
            return lasercan.intakedNote();
        }
        return false;
    }

    public double getMotorPosition() {
        if (attached) {
            return motor.getPosition().getValueAsDouble();
        }
        return 0;
    }

    @Override
    protected Config setConfig() {
        config = new FeederConfig();
        return config;
    }
}
