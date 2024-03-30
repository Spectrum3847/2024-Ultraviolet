package frc.robot.mechanisms.elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;
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
        public final double maxHeight = 29.8;
        public final double minHeight = 0;

        /* Elevator positions in rotations */
        public double fullExtend = maxHeight;
        public double home = minHeight;
        public double amp = 15;
        public double trap = 5;

        /* Elevator config settings */
        public final double zeroSpeed = -0.2;
        public final double positionKp = 0.86; // 20 FOC // 10 Regular
        public final double positionKv = 0.013; // .12 FOC // .15 regular
        public final double currentLimit = 20;
        public final double torqueCurrentLimit = 100;
        public final double threshold = 20;

        public ElevatorConfig() {
            super("Elevator", 52, "3847");
            configPIDGains(0, positionKp, 0, 0);
            configFeedForwardGains(0, positionKv, 0, 0);
            configMotionMagic(700, 900, 0); // 40, 120 FOC // 120, 195 Regular
            configSupplyCurrentLimit(currentLimit, threshold, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configForwardSoftLimit(maxHeight, true);
            configReverseSoftLimit(minHeight, true);
            configNeutralBrakeMode(true);
            // configMotionMagicPosition(0.12);
            configClockwise_Positive();
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

    // Is Amp Height
    public Boolean isAtAmpHeight() {
        return getMotorPosition() > config.amp * 0.8;
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

    /** Holds the position of the elevator. */
    public Command holdPosition() {
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
                holdPosition = getMotorPosition();
            }

            @Override
            public void execute() {
                double currentPosition = getMotorPosition();
                if (Math.abs(holdPosition - currentPosition) <= 5) {
                    setMMPosition(holdPosition);
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
                .withName("Elevator.zeroElevatorRoutine");
    }

    public boolean isElevatorUp() {
        if (attached) {
            return getMotorPosition() >= 5;
        }
        return false;
    }

    /* Logging */

    @AutoLogOutput(key = "Elevator/Position (rotations)")
    public double getMotorPosition() {
        if (attached) {
            return motor.getPosition().getValueAsDouble();
        }
        return 0;
    }

    @Override
    protected Config setConfig() {
        config = new ElevatorConfig();
        return config;
    }
}
