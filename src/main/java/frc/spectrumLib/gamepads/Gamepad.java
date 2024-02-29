package frc.spectrumLib.gamepads;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotTelemetry;
import java.util.function.DoubleSupplier;

public abstract class Gamepad extends SubsystemBase {

    public boolean configured = false;
    private boolean printed = false;
    public SpectrumController controller;
    private Rotation2d storedLeftStickDirection = new Rotation2d();
    private Rotation2d storedRightStickDirection = new Rotation2d();

    /**
     * Creates a new Gamepad.
     *
     * @param port The port the gamepad is plugged into
     * @param name The name of the gamepad
     * @param isXbox Xbox or PS5 controller
     * @param emulatedPS5Port emulated port for PS5 controller so we can rumble PS5 controllers.
     */
    public Gamepad(String name, int port, boolean isXbox, int emulatedPS5Port) {
        super(name);
        controller = new SpectrumController(port, isXbox, emulatedPS5Port);
    }

    @Override
    public void periodic() {
        configure();
    }

    // Configure the driver controller
    public void configure() {
        // Detect whether the xbox controller has been plugged in after start-up
        if (!configured) {
            boolean isConnected = controller.getHID().isConnected();
            if (!isConnected) {
                if (!printed) {
                    RobotTelemetry.print("##" + getName() + ": GAMEPAD NOT CONNECTED ##");
                    printed = true;
                }
                return;
            }

            // Configure button bindings once the driver controller is connected
            if (DriverStation.isTest()) {
                setupTestButtons();
            } else if (DriverStation.isDisabled()) {
                setupDisabledButtons();
            } else {
                setupTeleopButtons();
            }
            configured = true;

            RobotTelemetry.print("## " + getName() + ": gamepad is connected ##");
        }
    }

    // Reset the controller configure, should be used with
    // CommandScheduler.getInstance.clearButtons()
    // to reset buttons
    public void resetConfig() {
        configured = false;
        configure();
    }

    public double getTwist() {
        double right = controller.getRightTriggerAxis();
        double left = controller.getLeftTriggerAxis();
        double value = right - left;
        if (controller.getHID().isConnected()) {
            return value;
        }
        return 0;
    }

    /* Zero is stick up, 90 is stick to the left*/
    public Rotation2d getLeftStickDirection() {
        double x = -1 * controller.getLeftX();
        double y = -1 * controller.getLeftY();
        if (x != 0 || y != 0) {
            Rotation2d angle = new Rotation2d(y, x);
            storedLeftStickDirection = angle;
        }
        return storedLeftStickDirection;
    }

    public double getLeftStickCardinals() {
        double stickAngle = getLeftStickDirection().getRadians();
        if (stickAngle > -Math.PI / 4 && stickAngle <= Math.PI / 4) {
            return 0;
        } else if (stickAngle > Math.PI / 4 && stickAngle <= 3 * Math.PI / 4) {
            return Math.PI / 2;
        } else if (stickAngle > 3 * Math.PI / 4 || stickAngle <= -3 * Math.PI / 4) {
            return Math.PI;
        } else {
            return -Math.PI / 2;
        }
    }

    public double getLeftStickMagnitude() {
        double x = -1 * controller.getLeftX();
        double y = -1 * controller.getLeftY();
        return Math.sqrt(x * x + y * y);
    }

    public Rotation2d getRightStickDirection() {
        double x = controller.getRightX();
        double y = controller.getRightY();
        if (x != 0 || y != 0) {
            Rotation2d angle = new Rotation2d(y, x);
            storedRightStickDirection = angle;
        }
        return storedRightStickDirection;
    }

    /**
     * Get proper stick angles for each alliance
     *
     * @return
     */
    public double chooseCardinalDirections() {
        // hotfix
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return getRedAllianceStickCardinals();
        }
        return getBlueAllianceStickCardinals();
    }

    public double getBlueAllianceStickCardinals() {
        double stickAngle = getRightStickDirection().getRadians();
        if (stickAngle > -Math.PI / 4 && stickAngle <= Math.PI / 4) {
            return 0;
        } else if (stickAngle > Math.PI / 4 && stickAngle <= 3 * Math.PI / 4) {
            return Math.PI / 2;
        } else if (stickAngle > 3 * Math.PI / 4 || stickAngle <= -3 * Math.PI / 4) {
            return Math.PI;
        } else {
            return -Math.PI / 2;
        }
    }

    /**
     * Flips the stick direction for the red alliance.
     *
     * @return
     */
    public double getRedAllianceStickCardinals() {
        double stickAngle = getRightStickDirection().getRadians();
        if (stickAngle > -Math.PI / 4 && stickAngle <= Math.PI / 4) {
            return Math.PI;
        } else if (stickAngle > Math.PI / 4 && stickAngle <= 3 * Math.PI / 4) {
            return -Math.PI / 2;
        } else if (stickAngle > 3 * Math.PI / 4 || stickAngle <= -3 * Math.PI / 4) {
            return 0;
        } else {
            return Math.PI / 2;
        }
    }

    // public double getStickSteer(int segments){
    //     double stickAngle = getRightStickDirection().getRadians();
    //     double[] outputs = new double[segments];
    //     for (int i = 0; i < segments; i++){
    //         outputs[i] = i * (2*Math.PI/segments);
    //     }

    // }
    // public double idk (double[] outputs, double stickAngle, int segments){ ///need three parameters?, output array and current stickAngle in Radians, segments wanted
    //     double[] newoutputs = new double[segments]; //create copy of array with the rotated values
    //     for (int i=0; i < segments; i++){ // loops for the length of outputs
    //         newoutputs[i] = outputs[i] + Math.PI/segments; /// shifts circle
    //     }
    //     for (int i=0; i< segments; i++){
    //         if (stickAngle > outputs[i] && stickAngle <= outputs[(i+1)%segments]){ ///if the given stickAngle is between two of the other angles
    //             ///The %segments should account for the circle looping around?
    //             return outputs[(i+1)%segments];// shoudl be the desired angle output
    //         }
    //     }
        
    // }

    public double getRightStickMagnitude() {
        double x = controller.getRightX();
        double y = controller.getRightY();
        return Math.sqrt(x * x + y * y);
    }

    /** Setup modifier bumper and trigger buttons */
    public Trigger noBumpers() {
        return controller.rightBumper().negate().and(controller.leftBumper().negate());
    }

    public Trigger leftBumperOnly() {
        return controller.leftBumper().and(controller.rightBumper().negate());
    }

    public Trigger rightBumperOnly() {
        return controller.rightBumper().and(controller.leftBumper().negate());
    }

    public Trigger bothBumpers() {
        return controller.rightBumper().and(controller.leftBumper());
    }

    public Trigger noTriggers() {
        return controller.leftTrigger(0).negate().and(controller.rightTrigger(0).negate());
    }

    public Trigger leftTriggerOnly() {
        return controller.leftTrigger(0).and(controller.rightTrigger(0).negate());
    }

    public Trigger rightTriggerOnly() {
        return controller.rightTrigger(0).and(controller.leftTrigger(0).negate());
    }

    public Trigger bothTriggers() {
        return controller.leftTrigger(0).and(controller.rightTrigger(0));
    }

    public Trigger leftYTrigger(ThresholdType t, double threshold) {
        return axisTrigger(t, threshold, () -> controller.getLeftY());
    }

    public Trigger leftXTrigger(ThresholdType t, double threshold) {
        return axisTrigger(t, threshold, () -> controller.getLeftX());
    }

    public Trigger rightYTrigger(ThresholdType t, double threshold) {
        return axisTrigger(t, threshold, () -> controller.getRightY());
    }

    public Trigger rightXTrigger(ThresholdType t, double threshold) {
        return axisTrigger(t, threshold, () -> controller.getRightX());
    }

    public Trigger rightStick() {
        return new Trigger(
                () -> {
                    return Math.abs(controller.getRightX()) >= 0.1
                            || Math.abs(controller.getRightY()) >= 0.1;
                });
    }

    private Trigger axisTrigger(ThresholdType t, double threshold, DoubleSupplier v) {
        return new Trigger(
                () -> {
                    double value = v.getAsDouble();
                    switch (t) {
                        case GREATER_THAN:
                            return value > threshold;
                        case LESS_THAN:
                            return value < threshold;
                        case ABS_GREATER_THAN: // Also called Deadband
                            return Math.abs(value) > threshold;
                        default:
                            return false;
                    }
                });
    }

    public static enum ThresholdType {
        GREATER_THAN,
        LESS_THAN,
        ABS_GREATER_THAN;
    }

    private void rumble(double leftIntensity, double rightIntensity) {
        controller.rumbleController(leftIntensity, rightIntensity);
    }

    /** Command that can be used to rumble the pilot controller */
    public Command rumbleCommand(
            double leftIntensity, double rightIntensity, double durationSeconds) {
        return new RunCommand(() -> rumble(leftIntensity, rightIntensity), this)
                .withTimeout(durationSeconds)
                .ignoringDisable(true)
                .withName("Gamepad.Rumble");
    }

    public Command rumbleCommand(double intensity, double durationSeconds) {
        return rumbleCommand(intensity, intensity, durationSeconds);
    }

    /**
     * Run a command while a button/trigger is held down. Also runs a command for 1.5s when the
     * button/trigger is released.
     *
     * @param trigger
     * @param runCommand
     * @param endCommand
     */
    public void runWithEndSequence(Trigger trigger, Command runCommand, Command endCommand) {
        trigger.whileTrue(runCommand);
        trigger.onFalse(endCommand.withTimeout(1.5));
    }

    /**
     * Returns a new Command object that combines the given command with a rumble command. The
     * rumble command has a rumble strength of 1 and a duration of 0.5 seconds. The name of the
     * returned command is set to the name of the given command.
     *
     * @param command the command to be combined with the rumble command
     * @return a new Command object with rumble command
     */
    public Command rumbleCommand(Command command) {
        return command.alongWith(rumbleCommand(1, 0.5)).withName(command.getName());
    }

    public abstract void setupTeleopButtons();

    public abstract void setupDisabledButtons();

    public abstract void setupTestButtons();
}
