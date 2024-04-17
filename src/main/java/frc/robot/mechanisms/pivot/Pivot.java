package frc.robot.mechanisms.pivot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.crescendo.Field;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.RobotTelemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.mechanism.TalonFXFactory;
import frc.spectrumLib.swerve.config.SwerveConfig;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Pivot extends Mechanism {
    public class PivotConfig extends Config {

        /* Cancoder config */
        public final int CANcoderID = 44;
        public final double CANcoderGearRatio = 35.1;

        /* Pivot constants in motor rotations */
        public final double maxRotation = 0.96; // 0.967
        public final double minRotation = 0;

        /* Pivot positions in percentage of max rotation || 0 is horizontal */
        public final double score = 65;
        public final double climbHome = 3;
        public final double home = 1;
        public final double subwoofer = 81;
        public final double intoAmp = 78;
        public final double podium = 53.5;
        public final double fromAmp = 52;
        public final double ampWing = 41;
        public final double intake = 50;
        public final double manualFeed = 70;
        /* Auto Launch Positions */
        public final double autoLaunchPreload = 60;

        public final double autoLaunchPreload2 = 61;
        public final double spitReady = 5; // GP 4 on Front 6

        public final double spitReady2 = 32; // GP 4 on Front 6
        public final double autoLaunch1 = 70;
        public final double autoLaunch2 =
                55; // works for GP2/GP3/GP5/GP6 in Front 5 and GP2/GP4/5 in Front 5 Alt
        public final double autoLaunch3 = 54; // works for GP4 in Front 5 and GP3 in Front 5 Alt
        public final double autoLaunch4 = 50; // 5 and 6 on Front 6
        public final double autoLaunch5 = 74; // GP 2 on Front 6
        public final double autoLaunch6 = 45.5; // Gp 3 on Front 6
        public final double autoLaunch7 = 44.5; // GP 4 on Front 6
        public final double autoLaunch8 = 30; // GP 4 on Front 6
        public final double autoLaunch9 = 43.5; // GP 4 on Front 6

        public final double autoLaunch10 = 43.5;
        public final double autoLaunch11 = 54;
        public final double autoLaunch12 = 46;

        public final double autoLaunch13 = 46.5;

        public final double autoLaunch14 = 44.5;

        public final double autoLaunch15 = 43.5;

        public final double zeroSpeed = -0.1;

        /**
         * Percentage of pivot rotation added/removed from vision launching pivot angles (percentage
         * of the CHANGE in angle you set to, not +- the angle you set to) (the actual offset to
         * angles gets bigger as you get farther away)
         */
        public final double STARTING_OFFSET = 3;

        public double OFFSET = STARTING_OFFSET; // do not change this line

        public boolean shortFeed = true;

        /* Pivot config values */
        public double currentLimit = 30;
        public double torqueCurrentLimit = 100;
        public double threshold = 40;
        public double velocityKp = 186; // 200 w/ 0.013 good
        public double velocityKv = 0.018;
        public double velocityKs = 0;

        // TreeMap - Shooting position lookup table
        public static final InterpolatingDoubleTreeMap DISTANCE_MAP =
                new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap FEED_DISTANCE_MAP =
                new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap DEEP_FEED_DISTANCE_MAP =
                new InterpolatingDoubleTreeMap();

        static {
            // old map
            // DISTANCE_MAP.put(0.0, 82.0);
            // DISTANCE_MAP.put(1.505, 81.5);
            // DISTANCE_MAP.put(2.629, 57.5);
            // DISTANCE_MAP.put(2.942, 55.0);
            // DISTANCE_MAP.put(3.136, 53.0);
            // DISTANCE_MAP.put(3.396, 52.5);
            // DISTANCE_MAP.put(3.969, 50.0);
            // DISTANCE_MAP.put(4.269, 46.5);
            // DISTANCE_MAP.put(4.899, 45.0);
            // DISTANCE_MAP.put(5.189, 43.2);
            // DISTANCE_MAP.put(5.829, 42.0);
            // DISTANCE_MAP.put(6.229, 42.0);

            /* home */
            // 4500 RPM shots
            DISTANCE_MAP.put(1.5, 82.0);
            DISTANCE_MAP.put(1.7, 75.2);
            DISTANCE_MAP.put(1.9, 69.5);
            DISTANCE_MAP.put(2.1, 64.1);
            DISTANCE_MAP.put(2.3, 61.0);
            DISTANCE_MAP.put(2.5, 58.5);
            DISTANCE_MAP.put(2.7, 56.0);
            DISTANCE_MAP.put(2.9, 54.2);
            DISTANCE_MAP.put(3.1, 51.9);
            DISTANCE_MAP.put(3.3, 50.3);
            DISTANCE_MAP.put(3.5, 49.4);
            DISTANCE_MAP.put(3.7, 48.2);
            DISTANCE_MAP.put(3.9, 46.9);
            DISTANCE_MAP.put(4.1, 46.3);
            // 5000 RPM shots
            DISTANCE_MAP.put(4.11, 45.2);
            DISTANCE_MAP.put(4.2, 45.1);
            DISTANCE_MAP.put(4.3, 44.6);
            DISTANCE_MAP.put(4.4, 43.8);
            DISTANCE_MAP.put(4.5, 43.7);
            DISTANCE_MAP.put(4.6, 43.6);
            DISTANCE_MAP.put(4.8, 43.0);
            DISTANCE_MAP.put(5.0, 42.1);

            // feed -- OLD
            // FEED_DISTANCE_MAP.put(6.0, 69.0);
            // FEED_DISTANCE_MAP.put(6.08, 69.0);
            // FEED_DISTANCE_MAP.put(6.47, 69.0);
            // FEED_DISTANCE_MAP.put(6.96, 69.0);
            // FEED_DISTANCE_MAP.put(7.54, 68.0);
            // FEED_DISTANCE_MAP.put(7.74, 68.0);
            // FEED_DISTANCE_MAP.put(9.05, 65.0);

            // feed -- REVERT
            // FEED_DISTANCE_MAP.put(6.0, 82.0);
            // FEED_DISTANCE_MAP.put(6.08, 81.0);
            // FEED_DISTANCE_MAP.put(6.47, 75.0);
            // FEED_DISTANCE_MAP.put(6.96, 72.5);
            // FEED_DISTANCE_MAP.put(7.54, 70.0);
            // FEED_DISTANCE_MAP.put(7.74, 65.0);
            // FEED_DISTANCE_MAP.put(9.05, 65.0);
            FEED_DISTANCE_MAP.put(7.00, 81.0);

            DEEP_FEED_DISTANCE_MAP.put(7.0, 70.0);
        }

        public PivotConfig() {
            super("Pivot", 41, RobotConfig.CANIVORE);
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
            configMotionMagic(147000, 161000, 0);
        }
    }

    public PivotConfig config;
    private CANcoder m_CANcoder;

    public Pivot(boolean attached, SwerveConfig swerveConfig) {
        super(attached);
        if (attached) {
            modifyMotorConfig(swerveConfig); // Modify configuration to use remote CANcoder fused
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);
            m_CANcoder = new CANcoder(config.CANcoderID, RobotConfig.CANIVORE);
            CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
            cancoderConfigs.MagnetSensor.MagnetOffset = swerveConfig.pivotCANcoderOffset;
            cancoderConfigs.MagnetSensor.SensorDirection =
                    SensorDirectionValue.CounterClockwise_Positive;
            cancoderConfigs.MagnetSensor.AbsoluteSensorRange =
                    AbsoluteSensorRangeValue.Unsigned_0To1;
            checkMotorResponse(m_CANcoder.getConfigurator().apply(cancoderConfigs));
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
        return () -> getMapAngleAtSpeed(PivotConfig.FEED_DISTANCE_MAP, distance.getAsDouble(), 0);
        // return () -> PivotConfig.FEED_DISTANCE_MAP.get(distance.getAsDouble());
    }

    public DoubleSupplier getAngleFromDeepFeedDistance(DoubleSupplier distance) {
        return () ->
                getMapAngleAtSpeed(PivotConfig.DEEP_FEED_DISTANCE_MAP, distance.getAsDouble(), 0);
    }

    public static double getMapAngle(
            InterpolatingDoubleTreeMap map, double distance, double offset) {
        double angle = map.get(distance);
        angle += (angle * (offset / 100));
        RobotTelemetry.print(
                "VisionLaunching: interpolating "
                        + RobotTelemetry.truncatedDouble(angle)
                        + " percent rotation from "
                        + RobotTelemetry.truncatedDouble(distance)
                        + " meters");
        return angle;
    }

    public static double getMapAngleAtSpeed(
            InterpolatingDoubleTreeMap map, double distance, double offset) {
        double tunableSpeedFactor = 5;
        double angle = map.get(distance);
        angle += (angle * (offset / 100));
        double speed = Robot.swerve.getRobotRelativeSpeeds().vxMetersPerSecond;
        double speedOffset = speed * tunableSpeedFactor;
        angle += Field.isBlue() ? -speedOffset : speedOffset;
        RobotTelemetry.print(
                "VisionLaunching: interpolating "
                        + RobotTelemetry.truncatedDouble(angle)
                        + " percent rotation "
                        + " [original was "
                        + map.get(distance)
                        + "] from "
                        + RobotTelemetry.truncatedDouble(distance)
                        + " meters"
                        + " and detected X-speed: "
                        + speed);
        return Math.min(angle, 95);
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
                                                == NeutralModeValue.Coast)
                .ignoringDisable(true);
    }

    public Command zeroPivotRoutine() {
        return new FunctionalCommand(
                        () -> toggleReverseSoftLimit(false), // init
                        () -> setPercentOutput(config.zeroSpeed), // execute
                        (b) -> {
                            m_CANcoder.setPosition(0);
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

    public boolean pivotHasError() {
        if (attached) {
            return getMotorPosition() > config.maxRotation;
        }
        return false;
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

    @AutoLogOutput(key = "Pivot/Status")
    public boolean pivotStatus() {
        return !pivotHasError();
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

    public void switchFeedSpot() {
        config.shortFeed = !config.shortFeed;
        RobotTelemetry.print("Feed spot switched to " + ((config.shortFeed) ? " short" : " long"));
    }

    @AutoLogOutput(key = "Pivot/isShortFeed")
    public boolean isFeedShort() {
        return config.shortFeed;
    }

    public void checkMotorResponse(StatusCode response) {
        if (!response.isOK()) {
            System.out.println(
                    "Pivot CANcoder ID "
                            + config.CANcoderID
                            + " failed config with error "
                            + response.toString());
        }
    }

    public void modifyMotorConfig(SwerveConfig swerveConfig) {
        config.talonConfig.Feedback.FeedbackRemoteSensorID = config.CANcoderID;
        switch (swerveConfig.pivotFeedbackSource) {
            case RemoteCANcoder:
                config.talonConfig.Feedback.FeedbackSensorSource =
                        FeedbackSensorSourceValue.RemoteCANcoder;
                break;
            case FusedCANcoder:
                config.talonConfig.Feedback.FeedbackSensorSource =
                        FeedbackSensorSourceValue.FusedCANcoder;
                break;
            case SyncCANcoder:
                config.talonConfig.Feedback.FeedbackSensorSource =
                        FeedbackSensorSourceValue.SyncCANcoder;
                break;
        }
        config.talonConfig.Feedback.RotorToSensorRatio = config.CANcoderGearRatio;
    }

    @Override
    protected Config setConfig() {
        config = new PivotConfig();
        return config;
    }

    public enum CANCoderFeedbackType {
        RemoteCANcoder,
        FusedCANcoder,
        SyncCANcoder,
    }
}
