package frc.robot.mechanisms.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.mechanism.TalonFXFactory;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Pivot extends Mechanism {
    public class PivotConfig extends Config {

        /* Pivot constants in motor rotations */
        public final double maxRotation = 0; // starting at max pos
        // happen
        public final double minRotation = -23; // max motor rotations down
        public final double maxAngle = 70;
        public final double minAngle = 0;

        /* Pivot positions in angle measure(degrees) || 0 is min/flat */
        // 70% - max angle, 0% - min angle, these numbers are based on position
        public final double score = 56;
        public final double halfScore = 35;
        public final double test = 45.5;
        public final double home = 0;
        public final double fullangle = 70;
        public final double subwoofer = 45.5;

        public final double zeroSpeed = -0.2;

        /* Intake config values */
        public double currentLimit = 5;
        public double threshold = 5;
        public double velocityKp = 0.8;
        public double velocityKv = 0.013;
        public double velocityKs = 0;

        public PivotConfig() {
            super("Pivot", 41, "3847");
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1); // TODO: configure
            configSupplyCurrentLimit(currentLimit, threshold, true);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive(); // TODO: configure
            configReverseSoftLimit(minRotation, true);
            configForwardSoftLimit(maxRotation, true);
            configMotionMagic(51, 205, 0);
        }
    }

    public PivotConfig config;

    public Pivot(boolean attached) {
        super(attached);
        if (attached) {
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);
        }

        SmartDashboard.putNumber("pivotPercent", config.test);
    }

    @Override
    public void periodic() {}

    /**
     * Sets the intake motor to a specified position.
     *
     * @param angle angle of max degrees (70 is max). Note that the angle measure will be between 0
     *     and 70
     */
    public Command runPosition(double angle) {
        return run(() -> setMMPosition(angleToRotation(config.maxAngle - angle)))
                .withName("Pivot.runAngle");
    }

    /**
     * Sets the intake motor to a specified position using FOC control. Will require different PID
     * and Motion Magic gains
     *
     * @param percent percentage of max rotation (0 is vertical). Note that the percentage is not
     *     [-1,1] but rather [-100,100]
     */
    public Command runFOCPosition(double angle) {
        return run(() -> setMMPositionFOC(angleToRotation(config.maxAngle - angle))).withName("Pivot.runPercent");
    }

    /**
     * Runs the intake at a specified percentage of its maximum output.
     *
     * @param percent The percentage of the maximum output to run the intake at.
     */
    public Command runManualOutput(double percent) {
        return run(() -> setPercentOutput(percent)).withName("Pivot.runPercentage");
    }

    public Command runManualOutput(DoubleSupplier percentSupplier) {
        return run(() -> setPercentOutput(percentSupplier.getAsDouble()))
                .withName("Pivot.runPercentage");
    }

    /**
     * Stops the intake motor.
     *
     * @return
     */
    public Command runStop() {
        return run(() -> stop()).withName("Pivot.stop");
    }

    public Command coastMode() {
        return startEnd(() -> setBrakeMode(false), () -> setBrakeMode(true))
                .ignoringDisable(true)
                .withName("Pivot.coastMode");
    }

    /* Custom Commands */
    /** Holds the position of the pivot. */
    public Command runHoldPivot() { // TODO: review; inline custom commands vs. seperate class
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Pivot.holdPosition");
                addRequirements(Pivot.this);
            }

            @Override
            public void initialize() {
                holdPosition = getMotorPosition();
            }

            @Override
            public void execute() {
                setMMPosition(holdPosition);
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }
        };
    }

    /* Logging */

    /** Returns the position of the motor in rotations */
    @AutoLogOutput(key = "Pivot/Motor Position (rotations)")
    public double getMotorPosition() {
        if (attached) {
            return motor.getPosition().getValueAsDouble();
        }
        return 0;
    }

    /** Returns the position of the motor as a percentage of max rotation */
    @AutoLogOutput(key = "Pivot/Motor Position (percent)")
    public double getMotorPercentAngle() {
        if (attached) {
            return motor.getPosition().getValueAsDouble() / config.maxRotation * 100;
        }
        return 0;
    }

    // /* Helper */
    public double angleToRotation(double angle) {
        return config.minRotation * angle/config.maxAngle;
    }

    @Override
    protected Config setConfig() {
        config = new PivotConfig();
        return config;
    }
}
