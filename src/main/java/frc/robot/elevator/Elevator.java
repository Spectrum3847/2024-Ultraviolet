package frc.robot.elevator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.mechanism.TalonFXFactory;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Elevator extends Mechanism {
    public class ElevatorConfig extends Config {

        /* Elevator constants in rotations */
        public final double maxHeight = 10; //TODO: configure
        public final double minHeight = 0.29; //TODO: configure

        /* Elevator positions in rotations */
        public double fullExtend = maxHeight;
        public double home = minHeight;
        public double amp = 2; //TODO: configure
        public double trap = 5; //TODO: configure
        public double startingMotorPos = -0.15;

        /* Elevator config settings */
        public final double zeroSpeed = -0.2;
        public final double positionKp = 0.86; // 20 FOC // 10 Regular
        public final double positionKv = 0.013; // .12 FOC // .15 regular
        public final double currentLimit = 30;
        public final double threshold = 30;

        public ElevatorConfig() {
            super("Elevator", 52, "3847");
            configPIDGains(0, positionKp, 0, 0);
            configFeedForwardGains(0, positionKv, 0, 0);
            configMotionMagic(120, 195, 0); // 40, 120 FOC // 120, 195 Regular
            configSupplyCurrentLimit(currentLimit, threshold, true);
            configForwardSoftLimit(maxHeight, true);
            configReverseSoftLimit(minHeight, true);
            configNeutralBrakeMode(true);
            // configMotionMagicPosition(0.12);
            configClockwise_Positive(); //TODO: configure
        }
    }

    public ElevatorConfig config;

    public Elevator(boolean attached) {
        super(attached);
        if (attached) {
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);
        }
    }

    @Override
    public void periodic() {}

    /* Commands: see method in lambda for more information */

    /**
     * Runs the elevator to the specified position.
     *
     * @param position position in revolutions
     */
    public Command runPosition(double position) {
        return run(() -> setMMPosition(position)).withName("Elevator.runPosition");
    }

    /**
     * Runs the elevator to the specified position using FOC control. Will require different PID and
     * feedforward configs
     *
     * @param position position in revolutions
     */
    public Command runFOCPosition(double position) {
        return run(() -> setMMPositionFOC(position)).withName("Elevator.runFOCPosition");
    }

    /**
     * Runs the elevator at a specified percentage of its maximum output.
     *
     * @param percent fractional units between -1 and +1
     */
    public Command runPercentage(double percent) {
        return run(() -> setPercentOutput(percent)).withName("Elevator.runPercentage");
    }

    public Command runPercentage(DoubleSupplier percentSupplier) {
        return runPercentage(percentSupplier.getAsDouble());
    }

    // TODO: review; having commands in the elevator class would mean you are calling elevator commands from
    // two different places
    public Command runStop() {
        return run(() -> stop()).withName("Elevator.runStop");
    }

    /**
     * Temporarily sets the elevator to coast mode. The configuration is applied when the command is
     * started and reverted when the command is ended.
     */
    public Command coastMode() {
        return startEnd(() -> setBrakeMode(false), () -> setBrakeMode(true))
                .ignoringDisable(true)
                .withName("Elevator.coastMode");
    }

    /* Custom Commands */

    /** Holds the position of the elevator. */
    public Command holdPosition() { // TODO: review; inline custom commands vs. seperate class
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Elevator.holdPosition");
                addRequirements(Elevator.this);
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
                            holdPosition); // TODO: add: change mode depending on current control mode
                } else {
                    DriverStation.reportError(
                            "ElevatorHoldPosition tried to go too far away from current position. Current Position: "
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
    //TODO: fix: will not work currently
    public Command zeroElevatorRoutine() {
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
                .withName("Elevator.zeroElevatorRoutine");
    }

    /* Logging */

    @AutoLogOutput(key = "Elevator/Position (rotations)")
    public double getMotorPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    @Override
    protected Config setConfig() {
        config = new ElevatorConfig();
        return config;
    }
}
