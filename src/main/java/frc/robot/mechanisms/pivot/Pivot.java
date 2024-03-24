package frc.robot.mechanisms.pivot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotTelemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.mechanism.TalonFXFactory;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Pivot extends Mechanism {
    public class PivotConfig extends Config {

        /* Pivot constants in motor rotations */
        public final double maxRotation = 33.6;
        public final double minRotation = 0;

        /* Pivot positions in percentage of max rotation || 0 is horizontal */
        public final double score = 65;
        public final double home = 1;
        public final double subwoofer = 81;
        public final double intoAmp = 78;
        public final double podium = 53.5;
        public final double fromAmp = 52;
        public final double ampWing = 41;
        public final double intake = 50;
        public final double autoLaunchPreload = 60;
        public final double autoLaunch1 = 70;
        public final double autoLaunch2 =
                55; // works for GP2/GP3/GP5/GP6 in Front 5 and GP2/GP4/5 in Front 5 Alt
        public final double autoLaunch3 = 54; // works for GP4 in Front 5 and GP3 in Front 5 Alt
        public final double autoLaunch4 = 44; // 5 and 6 on Front 6
        public final double autoLaunch5 = 51; // GP 2 on Front 6
        public final double autoLaunch6 = 70; // Gp 3 on Front 6
        public final double autoLaunch7 = 52; // GP 4 on Front 6

        public final double zeroSpeed = -0.1;

        /** Percentage of pivot rotation added/removed from vision launching pivot angles */
        public final double STARTING_OFFSET = -1;

        public double OFFSET = STARTING_OFFSET; // do not change this

        /* Pivot config values */
        public double currentLimit = 30;
        public double torqueCurrentLimit = 100;
        public double threshold = 40;
        public double velocityKp = 0.8;
        public double velocityKv = 0.013;
        public double velocityKs = 0;

        // TreeMap - Shooting position lookup table
        public static final InterpolatingDoubleTreeMap DISTANCE_MAP =
                new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap FEED_DISTANCE_MAP =
                new InterpolatingDoubleTreeMap();

        static {
            // home
            DISTANCE_MAP.put(0.0, 82.0);
            DISTANCE_MAP.put(1.505, 81.5);
            DISTANCE_MAP.put(2.629, 57.5);
            DISTANCE_MAP.put(2.942, 54.5);
            DISTANCE_MAP.put(3.969, 49.0);
            DISTANCE_MAP.put(4.269, 46.5);
            DISTANCE_MAP.put(4.899, 45.0);
            DISTANCE_MAP.put(5.189, 43.2);
            DISTANCE_MAP.put(5.829, 42.0);
            DISTANCE_MAP.put(6.229, 42.0);

            // feed
            FEED_DISTANCE_MAP.put(6.0, 80.0);
            FEED_DISTANCE_MAP.put(6.08, 80.0);
            FEED_DISTANCE_MAP.put(6.47, 69.0);
            FEED_DISTANCE_MAP.put(6.96, 69.0);
            FEED_DISTANCE_MAP.put(7.54, 68.0);
            FEED_DISTANCE_MAP.put(7.74, 63.0);
            FEED_DISTANCE_MAP.put(9.05, 63.0);
        }

        public PivotConfig() {
            super("Pivot", 41, "3847");
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1);
            configSupplyCurrentLimit(currentLimit, threshold, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configNeutralBrakeMode(true);
            configClockwise_Positive();
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
            // motor.setPosition(config.maxRotation);
        }

        SmartDashboard.putNumber("pivotPercent", config.score);
    }

    @Override
    public void periodic() {}

    // Lookup angle in tree map, add fudge factor, and return angle
    public DoubleSupplier getAngleFromDistance(DoubleSupplier distance) {
        return () -> getMapAngle(PivotConfig.DISTANCE_MAP, distance.getAsDouble(), config.OFFSET);
        // return () -> (PivotConfig.DISTANCE_MAP.get(distance.getAsDouble()) + config.OFFSET);

    }

    public DoubleSupplier getAngleFromFeedDistance(DoubleSupplier distance) {
        return () ->
                getMapAngle(PivotConfig.FEED_DISTANCE_MAP, distance.getAsDouble(), config.OFFSET);
        // return () -> PivotConfig.FEED_DISTANCE_MAP.get(distance.getAsDouble());
    }

    public static double getMapAngle(
            InterpolatingDoubleTreeMap map, double distance, double offset) {
        double angle = map.get(distance) + offset;
        RobotTelemetry.print(
                "VisionLaunching: interpolating "
                        + RobotTelemetry.truncatedDouble(angle)
                        + " percent rotation from "
                        + RobotTelemetry.truncatedDouble(distance)
                        + " meters");
        return angle;
    }

    public Command runPosition(DoubleSupplier percent) {
        return run(() -> setMMPosition(percentToRotation(percent)))
                .withName("Pivot.runPercentSupplier");
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

    /** Stops the intake motor. */
    public Command runStop() {
        return run(() -> stop()).withName("Pivot.stop");
    }

    public Command coastMode() {
        return startEnd(() -> setBrakeMode(false), () -> setBrakeMode(true))
                .ignoringDisable(true)
                .withName("Pivot.coastMode");
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
                                                == NeutralModeValue.Coast);
    }

    public Command zeroElevatorRoutine() {
        return new FunctionalCommand(
                        () -> toggleReverseSoftLimit(false), // init
                        () -> setPercentOutput(config.zeroSpeed), // execute
                        (b) -> {
                            tareMotor();
                            toggleReverseSoftLimit(true); // end
                        },
                        () -> false, // isFinished
                        this) // requirement
                .withName("Pivot.zeroPivotRoutine");
    }

    /* Custom Commands */
    /** Holds the position of the pivot. */
    public Command runHoldPivot() {
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

    @AutoLogOutput(key = "Pivot/Adjustable Offset (percent)")
    public double getOffset() {
        if (attached) {
            return config.OFFSET;
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

    public void increaseOffset() {
        increaseOffset(1);
    }

    public void decreaseOffset() {
        decreaseOffset(1);
    }

    public void increaseOffset(double amount) {
        config.OFFSET += amount;
        RobotTelemetry.print("Pivot offset increased to: " + config.OFFSET);
    }

    public void decreaseOffset(double amount) {
        config.OFFSET -= amount;
        RobotTelemetry.print("Pivot offset decreased to: " + config.OFFSET);
    }

    public void resetOffset() {
        config.OFFSET = config.STARTING_OFFSET;
        RobotTelemetry.print("Pivot offset reset to: " + config.OFFSET);
    }

    @Override
    protected Config setConfig() {
        config = new PivotConfig();
        return config;
    }
}
