package frc.robot.mechanisms.climber;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotConfig;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.mechanism.TalonFXFactory;
import java.util.function.DoubleSupplier;

public class Climber extends Mechanism {
    public class ClimberConfig extends Config {

        /* Climber constants in rotations */
        public final double maxRotation = 104;
        public final double minRotation = -1;

        /* Climber positions in percent (0 - 100) of full rotation */
        public double fullExtend = 100;
        public double home = 0;

        public double topClimb = 100;
        public double midClimb = 74;
        public double safeClimb = 60;
        public double botClimb = 0;

        /* Climber Percentage Output */
        public double raisePercentage = 0.2;
        public double lowerPercentage = -0.2;

        /* Climber config settings */
        public final double zeroSpeed = -0.2;
        public final double positionKp = 1.3; // 20 FOC // 10 Regular
        public final double positionKv = 0.013; // .12 FOC // .15 regular
        public final double currentLimit = 80;
        public final double statorCurrentLimit = 200;
        public final double torqueCurrentLimit = 100;
        public final double threshold = 80;

        public ClimberConfig() {
            super("Climber", 53, RobotConfig.CANIVORE);
            configPIDGains(0, positionKp, 0, 0);
            configFeedForwardGains(0, positionKv, 0, 0);
            configMotionMagic(14700, 16100, 0); // 40, 120 FOC // 120, 195 Regular
            configSupplyCurrentLimit(currentLimit, threshold, true);
            configStatorCurrentLimit(statorCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configForwardSoftLimit(maxRotation, true);
            configReverseSoftLimit(minRotation, true);
            configNeutralBrakeMode(true);
            // configMotionMagicPosition(0.12);
            configClockwise_Positive();
        }
    }

    public ClimberConfig config;

    public Climber(boolean attached) {
        super(attached);
        if (attached) {
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);
        }
    }

    @Override
    public void periodic() {}

    /* Commands: see method in lambda for more information */

    /**
     * Runs the climber to the specified position.
     *
     * @param percent percentage of max rotation (0 is vertical). Note that the percentage is not
     *     [-1,1] but rather [-100,100]
     */
    public Command runPosition(double percent) {
        return run(() -> setMMPosition(percentToRotation(percent))).withName("Climber.runPosition");
    }

    /**
     * Runs the climber to the specified position using FOC control. Will require different PID and
     * feedforward configs
     *
     * @param position position in revolutions
     */
    public Command runFOCPosition(double position) {
        return run(() -> setMMPositionFOC(position)).withName("Climber.runFOCPosition");
    }

    /**
     * Runs the climber at a specified percentage of its maximum output.
     *
     * @param percent fractional units between -1 and +1
     */
    public Command runPercentage(double percent) {
        return run(() -> setPercentOutput(percent)).withName("Climber.runPercentage");
    }

    public Command runPercentage(DoubleSupplier percentSupplier) {
        return run(() -> setPercentOutput(percentSupplier.getAsDouble()));
    }

    public Command runStop() {
        return run(() -> stop()).withName("Climber.runStop");
    }

    /**
     * Temporarily sets the climber to coast mode. The configuration is applied when the command is
     * started and reverted when the command is ended.
     */
    public Command coastMode() {
        return startEnd(() -> setBrakeMode(false), () -> setBrakeMode(true))
                .ignoringDisable(true)
                .withName("Climber.coastMode");
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

    /* Custom Commands */

    /** Holds the position of the climber. */
    public Command holdPosition() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Climber.holdPosition");
                addRequirements(Climber.this);
            }

            @Override
            public void initialize() {
                stop();
                holdPosition = getMotorPosition();
            }

            @Override
            public void execute() {
                double currentPosition = getMotorPosition();
                if (Math.abs(holdPosition - currentPosition) <= 5) {
                    setMMPosition(holdPosition);
                } else {
                    DriverStation.reportError(
                            "ClimberHoldPosition tried to go too far away from current position. Current Position: "
                                    + currentPosition
                                    + " || Hold Position: "
                                    + holdPosition,
                            false);
                }
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }
        };
    }

    public Command zeroClimberRoutine() {
        return new FunctionalCommand(
                        () -> toggleReverseSoftLimit(false), // init
                        () -> setPercentOutput(config.zeroSpeed), // execute
                        (b) -> {
                            tareMotor();
                            toggleReverseSoftLimit(true); // end
                        },
                        () -> false, // isFinished
                        this) // requirement
                .withName("Climber.zeroClimberRoutine");
    }

    /* Helper */

    public double percentToRotation(double percent) {
        return config.maxRotation * (percent / 100);
    }

    public DoubleSupplier percentToRotation(DoubleSupplier percent) {
        return () -> config.maxRotation * (percent.getAsDouble() / 100);
    }

    /* Logging */

    // @AutoLogOutput(key = "Climber/Position (rotations)")
    public double getMotorPosition() {
        if (attached) {
            return motor.getPosition().getValueAsDouble();
        }
        return 0;
    }

    /** Returns the position of the motor as a percentage of max rotation */
    // @AutoLogOutput(key = "Climber/Motor Position (percent)")
    public double getMotorPercentAngle() {
        if (attached) {
            return motor.getPosition().getValueAsDouble() / config.maxRotation * 100;
        }
        return 0;
    }

    @Override
    protected Config setConfig() {
        config = new ClimberConfig();
        return config;
    }
}
