package frc.robot.mechanisms.amptrap;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConfig;
import frc.spectrumLib.lasercan.LaserCanUtil;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.mechanism.TalonFXFactory;
import frc.spectrumLib.util.Conversions;
import org.littletonrobotics.junction.AutoLogOutput;

public class AmpTrap extends Mechanism {
    public class AmpTrapConfig extends Config {

        /* Revolutions per min AmpTrap Output */
        public double maxSpeed = 5000;
        public double intake = 3000;
        public double feed = 500;
        public double amp = 4500;
        public double score = 4500;
        public double eject = -3000;

        /* Percentage AmpTrap Output */
        public double slowIntakePercentage = 0.1;

        /* AmpTrap config values */
        public double currentLimit = 30;
        public double torqueCurrentLimit = 100;
        public double threshold = 40;
        public double velocityKp = 0.156152;
        public double velocityKv = 0.12;
        public double velocityKs = 0.24;

        public double hasNoteDistance = 300;
        public double topHasNoteDistance = 150;

        public AmpTrapConfig() {
            super("AmpTrap", 51, RobotConfig.RIO_CANBUS);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(12 / 30);
            configSupplyCurrentLimit(currentLimit, threshold, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
            configMotionMagic(51, 205, 0);
        }
    }

    public AmpTrapConfig config;
    public LaserCanUtil bottomLasercan;
    public LaserCanUtil topLasercan;

    public AmpTrap(boolean attached) {
        super(attached);
        if (attached) {
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);
        }

        bottomLasercan = new LaserCanUtil(1);
        topLasercan = new LaserCanUtil(2);
    }

    @AutoLogOutput(key = "AmpTrap/Lasercans/Bottom/Measurement")
    public int getBotLaserCanDistance() {
        return bottomLasercan.getDistance();
    }

    @AutoLogOutput(key = "AmpTrap/Lasercans/Top/Measurement")
    public int getTopLaserCanDistance() {
        return topLasercan.getDistance();
    }

    @AutoLogOutput(key = "AmpTrap/Lasercans/Bottom/LaserCan-Valid")
    public boolean getBotLaserCanStatus() {
        return bottomLasercan.validDistance();
    }

    @AutoLogOutput(key = "AmpTrap/Lasercans/Top/LaserCan-Valid")
    public boolean getTopLaserCanStatus() {
        return topLasercan.validDistance();
    }

    public boolean bottomHasNote() {
        if (getBotLaserCanDistance() <= 0) {
            return false;
        }
        return getBotLaserCanDistance() < config.hasNoteDistance;
    }

    public boolean topHasNote() {
        if (getTopLaserCanDistance() <= 0) {
            return false;
        }
        return getTopLaserCanDistance() < config.topHasNoteDistance;
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

    public Command stayCoastMode() {
        return runOnce(() -> setBrakeMode(false))
                .ignoringDisable(true)
                .withName("AmpTrap.stayCoastMode");
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
