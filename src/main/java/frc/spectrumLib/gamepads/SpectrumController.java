package frc.spectrumLib.gamepads;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Handles the conversion from Xbox controller to PS5 Controller. Basic control of the controllers.
 * Do not put modifiers or custom triggers in here. use {@link Gamepad} instead
 */
public class SpectrumController {
    private boolean isXbox;
    private CommandXboxController xboxController;
    private CommandPS5Controller ps5Controller;
    private CommandGenericHID abstractedController;
    private CommandXboxController emulatedController;

    public SpectrumController(int port, boolean isXbox, int emulatedPort) {
        this.isXbox = isXbox;
        if (isXbox) {
            xboxController = new CommandXboxController(port);
            abstractedController = xboxController;
        } else {
            ps5Controller = new CommandPS5Controller(port);
            emulatedController = new CommandXboxController(emulatedPort);
            abstractedController = ps5Controller;
        }
    }

    /* Basic Controller Buttons */

    public Trigger a() {
        return (isXbox) ? xboxController.a() : ps5Controller.cross();
    }

    public Trigger b() {
        return (isXbox) ? xboxController.b() : ps5Controller.circle();
    }

    public Trigger x() {
        return (isXbox) ? xboxController.x() : ps5Controller.square();
    }

    public Trigger y() {
        return (isXbox) ? xboxController.y() : ps5Controller.triangle();
    }

    public Trigger leftBumper() {
        return (isXbox) ? xboxController.leftBumper() : ps5Controller.L1();
    }

    public Trigger rightBumper() {
        return (isXbox) ? xboxController.rightBumper() : ps5Controller.R1();
    }

    public Trigger leftTrigger(double threshold) {
        return (isXbox)
                ? xboxController.leftTrigger(threshold)
                : new Trigger(
                        () -> Math.min(Math.abs(ps5Controller.getL2Axis()), 0.99) > threshold);
    }

    public Trigger rightTrigger(double threshold) {
        return (isXbox)
                ? xboxController.rightTrigger(threshold)
                : new Trigger(
                        () -> Math.min(Math.abs(ps5Controller.getR2Axis()), 0.99) > threshold);
    }

    /* Left stick is PRESSED DOWN (is activated by pressing the right stick into the gamepad until it clicks) */
    public Trigger leftStick() {
        return (isXbox) ? xboxController.leftStick() : ps5Controller.L3();
    }

    /* Right stick is PRESSED DOWN (is activated by pressing the right stick into the gamepad until it clicks) */
    public Trigger rightStick() {
        return (isXbox) ? xboxController.rightStick() : ps5Controller.R3();
    }

    /* On XBox, it is the button left of the X button and on PS5 it is the button to the right of the trackpad */
    public Trigger start() {
        return (isXbox) ? xboxController.start() : ps5Controller.options();
    }

    /* On XBox, it is the button right of the left joystick and on PS5 it is the button to the left of the trackpad */
    public Trigger select() {
        return (isXbox) ? xboxController.back() : ps5Controller.create();
    }

    /** PS5 only */
    public Trigger trackpad() {
        return (isXbox) ? new Trigger(() -> false) : ps5Controller.touchpad();
    }

    public Trigger upDpad() {
        return abstractedController.povUp();
    }

    public Trigger downDpad() {
        return abstractedController.povDown();
    }

    public Trigger leftDpad() {
        return abstractedController.povLeft();
    }

    public Trigger rightDpad() {
        return abstractedController.povRight();
    }

    public double getRightTriggerAxis() {
        return (isXbox)
                ? xboxController.getRightTriggerAxis()
                : Math.min(Math.abs(ps5Controller.getR2Axis()), 0.99);
    }

    public double getLeftTriggerAxis() {
        return (isXbox)
                ? xboxController.getLeftTriggerAxis()
                : Math.min(Math.abs(ps5Controller.getL2Axis()), 0.99);
    }

    public double getLeftX() {
        return (isXbox) ? xboxController.getLeftX() : ps5Controller.getLeftX();
    }

    public double getLeftY() {
        return (isXbox) ? xboxController.getLeftY() : ps5Controller.getLeftY();
    }

    public double getRightX() {
        return (isXbox) ? xboxController.getRightX() : ps5Controller.getRightX();
    }

    public double getRightY() {
        return (isXbox) ? xboxController.getRightY() : ps5Controller.getRightY();
    }

    public GenericHID getHID() {
        return (isXbox) ? xboxController.getHID() : ps5Controller.getHID();
    }

    public GenericHID getRumbleHID() {
        return (isXbox) ? xboxController.getHID() : emulatedController.getHID();
    }

    public void rumbleController(double leftIntensity, double rightIntensity) {
        getRumbleHID().setRumble(RumbleType.kLeftRumble, leftIntensity);
        getRumbleHID().setRumble(RumbleType.kRightRumble, rightIntensity);
    }
}
