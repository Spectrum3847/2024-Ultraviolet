package frc.robot.mechanisms.pivot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.mechanism.TalonFXFactory;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Pivot extends Mechanism {
    public class PivotConfig extends Config {

        /* Pivot constants in motor rotations */
        public final double maxRotation =
                43.5 * 3 * 25 / 36; // furthest it goes up, also is amp score
        public final double minRotation = 0;

        /* Pivot positions in percentage of max rotation || 0 is horizontal */
        public final int score = 65;
        public final int halfScore = 50;
        public final int test = 36;
        public final int home = 0;
        public final int subwoofer = 10;
        public final int podium = 36;
        public final int amp = 85;
        public final int autoLaunchPreload = 23;
        public final int autoLaunch2 =
                27; // works for GP2/GP3/GP5/GP6 in Front 5 and GP2/GP4/5 in Front 5 Alt
        public final int autoLaunch3 = 36; // works for GP4 in Front 5 and GP3 in Front 5 Alt
        public final int climb = 130; // 120

        public final double zeroSpeed = -0.2;

        public double offset = 0;

        /* Intake config values */
        public double currentLimit = 30;
        public double threshold = 40;
        public double velocityKp = 0.8;
        public double velocityKv = 0.013;
        public double velocityKs = 0.5;

        public static final InterpolatingDoubleTreeMap DISTANCE_MAP =
                new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap FEED_DISTANCE_MAP =
                new InterpolatingDoubleTreeMap();

        static {
            // home
            DISTANCE_MAP.put(1.18, 10.0);
            DISTANCE_MAP.put(1.76, 24.0);
            DISTANCE_MAP.put(2.79, 34.5);
            DISTANCE_MAP.put(2.93, 37.0);
        }

        public PivotConfig() {
            super("Pivot", 41, "3847");
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1); // TODO: configure
            configSupplyCurrentLimit(currentLimit, threshold, true);
            configNeutralBrakeMode(true);
            configClockwise_Positive(); // TODO: configure
            configReverseSoftLimit(minRotation, true);
            configForwardSoftLimit(maxRotation + 30.0, true); // + 20
            configMotionMagic(100, 205, 0);
        }
    }

    public PivotConfig config;
    //
    public Pivot(boolean attached) {
        super(attached);
        if (attached) {
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);
        }

        SmartDashboard.putNumber("pivotPercent", config.test);
    }

    @Override
    public void periodic() {}

    // Lookup angle in tree map, add fudge factor, and return angle
    public DoubleSupplier getAngleFromDistance(DoubleSupplier distance) {
        return () -> (PivotConfig.DISTANCE_MAP.get(distance.getAsDouble()) + config.offset);
    }

    /**
     * Sets the intake motor to a specified position.
     *
     * @param percent percentage of max rotation (0 is vertical). Note that the percentage is not
     *     [-1,1] but rather [-100,100]
     */
    public Command runPosition(double percent) {
        return run(() -> setMMPosition(percentToRotation(percent))).withName("Pivot.runPercent");
    }

    public Command runPosition(DoubleSupplier percent) {
        return run(() -> setMMPosition(percentToRotation(percent))).withName("Pivot.runPercent");
    }

    /**
     * Sets the intake motor to a specified position using FOC control. Will require different PID
     * and Motion Magic gains
     *
     * @param percent percentage of max rotation (0 is vertical). Note that the percentage is not
     *     [-1,1] but rather [-100,100]
     */
    public Command runFOCPosition(double percent) {
        return run(() -> setMMPositionFOC(percentToRotation(percent))).withName("Pivot.runPercent");
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

    /* Helper */
    public double percentToRotation(double percent) {
        return config.maxRotation * (percent / 100);
    }

    public DoubleSupplier percentToRotation(DoubleSupplier percent) {
        return () -> config.maxRotation * (percent.getAsDouble() / 100);
    }

    @Override
    protected Config setConfig() {
        config = new PivotConfig();
        return config;
    }
}
