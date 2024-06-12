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

/** Top Launcher */
public class RightLauncher extends Mechanism {
    public class RightLauncherConfig extends Config {

        /* Revolutions per min RightLauncher Output */
        public double maxSpeed = 5600;
        public double launch = 4000;
        public double testVelocity = 4500;
        public double subwoofer = 4500;
        public double deepShot = 5400;
        public double intoAmp = 650;
        public double manualSource = -2000;
        public double autoShoot = 5500;
        public double manualFeed = 4000;

        /* Percentage LeftLauncher Output */
        public double slowLeftLauncherPercent = 0.06;
        public double ejectLauncherPercent = -0.3;
        public double dumpLauncherPercent = 0.13;
        public double autoDumpLauncherPercent = 0.07;
        public double auto2DumpLauncherPercent = 0.3;
        public double auto3DumpLauncherPercent = 0.17;
        public double auto4DumpLauncherPercent = 0.185;

        /* RightLauncher config values */
        public double currentLimit = 60;
        public double torqueCurrentLimit = 300;
        public double threshold = 80;
        public double velocityKp = 4; // 12;
        public double velocityKv = 0.2; // 0.12;
        public double velocityKs = 14;

        public RightLauncherConfig() {
            super("RightLauncher", 43, RobotConfig.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1 / 2);
            configSupplyCurrentLimit(currentLimit, threshold, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configNeutralBrakeMode(true);
            configClockwise_Positive();
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
     * Run the right launcher at given velocity in TorqueCurrentFOC mode
     *
     * @param percent
     * @return
     */
    public Command runVelocityTCFOC(double velocity) {
        return run(() -> setVelocityTorqueCurrentFOC(Conversions.RPMtoRPS(velocity)))
                .withName("RightLauncher.runVelocityFOC");
    }

    /**
     * Run the right launcher at given velocityRPM in TorqueCurrentFOC mode
     *
     * @param percent
     * @return
     */
    public Command runVelocityTCFOCrpm(DoubleSupplier velocity) {
        return run(() -> setVelocityTCFOCrpm(velocity)).withName("RightLauncher.runVelocityFOC");
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
