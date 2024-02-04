package frc.robot.mechanisms.launcher;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.mechanism.TalonFXFactory;
import frc.spectrumLib.util.Conversions;
import org.littletonrobotics.junction.AutoLogOutput;

// TODO: this will get merged with left launcher into one class later
public class RightLauncher extends Mechanism {
    public class RightLauncherConfig extends Config {

        /* Revolutions per min RightLauncher Output */
        public double maxSpeed = 5000; // TODO: configure
        public double launch = 4000; // TODO: configure
        public double testVelocity = 4500;
        public double subwoofer = 4500;

        /* Percentage RightLauncher Output */
        public double slowRightLauncherPercentage = 0.06; // TODO: configure
        public double testForwardPercent = 0.5;
        public double testBackPercent = -0.5;

        /* RightLauncher config values */
        public double currentLimit = 40;
        public double threshold = 80;
        public double velocityKp = 0.156152;
        public double velocityKv = 0.12;
        public double velocityKs = 0.24;

        public RightLauncherConfig() {
            super("RightLauncher", 43, "3847");
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1 / 2); // TODO: configure
            configSupplyCurrentLimit(currentLimit, threshold, false);
            configNeutralBrakeMode(true);
            configClockwise_Positive(); // TODO: configure
            configMotionMagic(51, 205, 0);
        }
    }

    public RightLauncherConfig config;

    public RightLauncher(boolean attached) {
        super(attached);
        if (attached) {
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);

            SmartDashboard.putNumber("rightLaunchSpeed", config.testVelocity);
        }
    }

    @Override
    public void periodic() {}

    /* Control methods: see method in lambda for more information */

    /**
     * Runs the right launcher at a given velocity
     *
     * @param velocity in revolutions per minute
     */
    public Command runVelocity(double velocity) {
        return run(() -> setVelocity(Conversions.RPMtoRPS(velocity)))
                .withName("RightLauncher.runVelocity");
    }

    /**
     * Runs the right launcher at a specified percentage of its maximum output.
     *
     * @param percent The percentage of the maximum output to run the right launcher at.
     */
    public Command runPercentage(double percent) {
        return run(() -> setPercentOutput(percent)).withName("RightLauncher.runPercentage");
    }

    /**
     * Stops the right launcher motor.
     *
     * @return
     */
    public Command runStop() {
        return run(() -> stop()).withName("RightLauncher.stop");
    }

    /**
     * Temporarily sets the right launcher to coast mode. The configuration is applied when the
     * command is started and reverted when the command is ended.
     */
    public Command coastMode() {
        return startEnd(() -> setBrakeMode(false), () -> setBrakeMode(true))
                .ignoringDisable(true)
                .withName("RightLauncher.coastMode");
    }

    /* Logging */

    /** Returns the velocity of the motor in rotations per second */
    @AutoLogOutput(key = "RightLauncher/Motor Velocity (rotations per second)")
    public double getMotorVelocity() {
        if (attached) {
            return motor.getVelocity().getValueAsDouble();
        }
        return 0;
    }

    /** Returns the velocity of the motor in rotations per second */
    @AutoLogOutput(key = "RightLauncher/Motor Velocity (revolutions per minute)")
    public double getMotorVelocityInRPM() {
        if (attached) {
            return Conversions.RPStoRPM(getMotorVelocity());
        }
        return 0;
    }

    @Override
    protected Config setConfig() {
        config = new RightLauncherConfig();
        return config;
    }
}
