package frc.robot.mechanisms.climber;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.mechanism.TalonFXFactory;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

// TODO: this mechanism's motor has not been setup (given id and updated)
public class Climber extends Mechanism {
    public class ClimberConfig extends Config {

        /* Climber constants in rotations */
        public final double maxHeight = 147; // TODO: configure
        public final double minHeight = 0.5; // TODO: configure

        /* Climber positions in rotations */
        public double fullExtend = maxHeight;
        public double home = minHeight;
        public double topClimb = maxHeight;
        public double midClimb = 74;
        public double botClimb = minHeight;

        /* Climber Percentage Output */
        public double raisePercentage = 0.2;
        public double lowerPercentage = -0.2;

        /* Climber config settings */
        public final double zeroSpeed = -0.2;
        public final double positionKp = 0.86; // 20 FOC // 10 Regular
        public final double positionKv = 0.013; // .12 FOC // .15 regular
        public final double currentLimit = 30;
        public final double threshold = 30;

        public ClimberConfig() {
            super("Climber", 53, "3847"); // TODO: configure ID
            configPIDGains(0, positionKp, 0, 0);
            configFeedForwardGains(0, positionKv, 0, 0);
            configMotionMagic(120, 195, 0); // 40, 120 FOC // 120, 195 Regular
            configSupplyCurrentLimit(currentLimit, threshold, true);
            configForwardSoftLimit(maxHeight, true);
            configReverseSoftLimit(minHeight, true);
            configNeutralBrakeMode(true);
            // configMotionMagicPosition(0.12);
            configClockwise_Positive(); // TODO: configure
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
     * @param position position in revolutions
     */
    public Command runPosition(double position) {
        return run(() -> setMMPosition(position)).withName("Climber.runPosition");
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

    // TODO: review; having commands in the climber class would mean you are calling climber
    // commands from
    // two different places
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

    /* Custom Commands */

    /** Holds the position of the climber. */
    public Command holdPosition() { // TODO: review; inline custom commands vs. seperate class
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
                holdPosition = motor.getPosition().getValueAsDouble();
            }

            @Override
            public void execute() {
                double currentPosition = motor.getPosition().getValueAsDouble();
                if (Math.abs(holdPosition - currentPosition) <= 5) {
                    setMMPosition(
                            holdPosition); // TODO: add: change mode depending on current control
                    // mode
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

    // TODO: review; inline vs custom command
    // TODO: fix: will not work currently
    public Command zeroClimberRoutine() {
        return new FunctionalCommand( // TODO: refresh config in order to modify soft limits
                        () ->
                                config.configReverseSoftLimit(
                                        config.talonConfig
                                                .SoftwareLimitSwitch
                                                .ReverseSoftLimitThreshold,
                                        false),
                        () -> setPercentOutput(config.zeroSpeed),
                        (b) -> {
                            zeroMotor();
                            config.configReverseSoftLimit(
                                    config.talonConfig
                                            .SoftwareLimitSwitch
                                            .ReverseSoftLimitThreshold,
                                    true);
                        },
                        () -> false,
                        this)
                .withName("Climber.zeroClimberRoutine");
    }

    /* Logging */

    @AutoLogOutput(key = "Climber/Position (rotations)")
    public double getMotorPosition() {
        if (attached) {
            return motor.getPosition().getValueAsDouble();
        }
        return 0;
    }

    @Override
    protected Config setConfig() {
        config = new ClimberConfig();
        return config;
    }
}
