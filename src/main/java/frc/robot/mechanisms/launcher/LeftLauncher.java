package frc.robot.mechanisms.launcher;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConfig;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.mechanism.TalonFXFactory;
import frc.spectrumLib.util.Conversions;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

/** Bottom Launcher */
public class LeftLauncher extends Mechanism {
    public class LeftLauncherConfig extends Config {

        /* Revolutions per min LeftLauncher Output */
        public double maxSpeed = 5600;
        public double launch = 4000;
        public double testVelocity = 4500;
        public double ampVelocity = 2250;
        public double subwoofer = 4500;
        public double intoAmp = 1000;
        public double manualFeed = 4000;
        public double manualSource = -2000;
        public double launchReadyPreload = 4500;
        public double launchReady2 = 4500;
        public double launchReady3 = 4500;

        /* Percentage LeftLauncher Output */
        public double slowLeftLauncherPercent = 0.06;
        public double ejectLauncherPercent = -0.3;
        public double dumpLauncherPercent = 0.13;
        public double autoDumpLauncherPercent = 0.07;
        public double auto2DumpLauncherPercent = 0.3;
        public double auto3DumpLauncherPercent = 0.15;
        public double auto4DumpLauncherPercent = 0.2;

        /* LeftLauncher config values */
        public double currentLimit = 60;
        public double torqueCurrentLimit = 300;
        public double threshold = 80;
        public double velocityKp = 6;
        public double velocityKv = 0.12;
        public double velocityKs = 0.24;

        public LeftLauncherConfig() {
            super("LeftLauncher", 42, RobotConfig.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1 / 2); // TODO: configure
            configSupplyCurrentLimit(currentLimit, threshold, false);
            configNeutralBrakeMode(true);
            configClockwise_Positive(); // TODO: configure
            configMotionMagic(51, 205, 0);
        }
    }

    public LeftLauncherConfig config;

    public LeftLauncher(boolean attached) {
        super(attached);
        if (attached) {
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);

            SmartDashboard.putNumber("leftLaunchSpeed", config.testVelocity);
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
     * Run the left launcher at given velocity in TorqueCurrentFOC mode
     *
     * @param percent
     * @return
     */
    public Command runVelocityTCFOC(double velocity) {
        return run(() -> setVelocityTorqueCurrentFOC(Conversions.RPMtoRPS(velocity)))
                .withName("LeftLauncher.runVelocityFOC");
    }

    /**
     * Run the left launcher at given velocityRPM in TorqueCurrentFOC mode
     *
     * @param percent
     * @return
     */
    public Command runVelocityTCFOCrpm(DoubleSupplier velocity) {
        return run(() -> setVelocityTorqueCurrentFOC(velocity))
                .withName("LeftLauncher.runVelocityFOC");
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
     * Temporarily sets the left launcher to coast mode. The configuration is applied when the
     * command is started and reverted when the command is ended.
     */
    public Command coastMode() {
        return startEnd(() -> setBrakeMode(false), () -> setBrakeMode(true))
                .ignoringDisable(true)
                .withName("LeftLauncher.coastMode");
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
