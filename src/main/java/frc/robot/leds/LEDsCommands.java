package frc.robot.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.leds.LEDsConfig.Section;
import frc.robot.vision.Vision;

public class LEDsCommands {
    private static LEDs leds = Robot.leds;

    public static void setupDefaultCommand() {
        leds.setDefaultCommand(defaultCommand());
    }

    public static void setupLEDTriggers() {
        Trigger visionValid = new Trigger(() -> Vision.isPresent);
        Trigger noteIntaked = new Trigger(Robot.feeder.lasercan::intakedNote);
        visionValid.whileTrue(solidGreenLED());
        noteIntaked.whileTrue(whiteBounce());
    }

    /* Default Command */

    public static Command defaultCommand() {
        return leds.run(leds::defaultPattern)
                .finallyDo((b) -> leds.resetPriority())
                .ignoringDisable(true)
                .withName("LEDs.default");
    }

    /* Specific Commands */

    public static Command solidWhiteLED() {
        return LEDsCommands.solid(Section.FULL, Color.kWhite, 1).withName("LEDs.solidWhiteLED");
    }

    public static Command solidPurpleLED() {
        return LEDsCommands.solid(Section.FULL, Color.kPurple, 2).withName("LEDs.solidPurpleLED");
    }

    public static Command solidGreenLED() {
        return LEDsCommands.solid(Section.FULL, Color.kGreen, 3).withName("LEDs.solidGreenLED");
    }

    public static Command exampleOmbre() {
        return ombre(Section.FULL, new Color(130, 103, 185), Color.kWhite, 2);
    }

    public static Command exampleGradient() {
        return staticGradient(Section.FULL, Color.kWhite, new Color(130, 103, 185), 2);
    }

    public static Command exampleBounce() {
        return bounce(
                Section.FULL,
                new Color(130, 103, 185),
                new Color(80, 53, 135),
                new Color(30, 3, 85),
                Color.kBlack,
                0.58,
                0);
    }

    public static Command whiteBounce() {
        return bounce(
                Section.FULL,
                new Color(255, 255, 255),
                new Color(205, 205, 205),
                new Color(155, 155, 155),
                Color.kBlack,
                0.58,
                3);
    }

    public static Command strobeOrangeLED() {
        return LEDsCommands.strobe(Section.FULL, Color.kOrange, 0.5, 2)
                .withName("LEDs.strobeOrangeLED");
    }

    public static Command breathBlueLED() {
        return LEDsCommands.breath(Section.FULL, Color.kBlue, Color.kBlack, 1, 4)
                .withName("LEDs.breathBlueLED");
    }

    /*
     *
     * Helper LED commands
     *
     *
     */

    /* Solid */

    public static Command solid(Color color, int priority) {
        return solid(Section.FULL, color, priority);
    }

    public static Command solid(Section section, Color color, int priority) {
        return runLEDPattern(() -> leds.solid(section, color, priority)).withName("LEDs.solid");
    }

    public static Command solid(double percent, Color color, int priority) {
        return runLEDPattern(() -> leds.solid(percent, color, priority)).withName("LEDs.solid");
    }

    /* Strobe */

    public static Command strobe(Color color, int priority) {
        return strobe(Section.FULL, color, 0.5, priority);
    }

    public static Command strobe(Section section, Color color, double duration, int priority) {
        return runLEDPattern(() -> leds.strobe(section, color, duration, priority))
                .withName("LEDs.strobe");
    }

    /* Breath */

    public static Command breath(Color c1, Color c2, int priority) {
        return breath(Section.FULL, c1, c2, 1, priority);
    }

    public static Command breath(
            Section section, Color c1, Color c2, double duration, int priority) {
        return runLEDPattern(() -> leds.breath(section, c1, c2, duration, priority))
                .withName("LEDs.breath");
    }

    /* Rainbow */

    public static Command rainbow(int priority) {
        return rainbow(Section.FULL, LEDsConfig.length, 1, priority);
    }

    public static Command rainbow(
            Section section, double cycleLength, double duration, int priority) {
        return runLEDPattern(() -> leds.rainbow(section, cycleLength, duration, priority))
                .withName("LEDs.rainbow");
    }

    /* Wave */

    public static Command wave(Color c1, Color c2, int priority) {
        return wave(Section.FULL, c1, c2, LEDsConfig.length, 1, priority);
    }

    /* Ombre */

    public static Command ombre(Section section, Color c1, Color c2, int priority) {
        return runLEDPattern(() -> leds.ombre(section, c1, c2, priority)).withName("LEDs.ombre");
    }

    /* Gradient */

    public static Command staticGradient(Section section, Color c1, Color c2, int priority) {
        return runLEDPattern(() -> leds.staticGradient(section, c1, c2, priority))
                .withName("LEDs.staticGradient");
    }

    /* Bounce */

    public static Command bounce(
            Section section,
            Color primaryColor,
            Color secondaryColor,
            Color tertiaryColor,
            Color backgroundColor,
            double speed,
            int priority) {
        return runLEDPattern(
                        () ->
                                leds.bounce(
                                        section,
                                        primaryColor,
                                        secondaryColor,
                                        tertiaryColor,
                                        backgroundColor,
                                        speed,
                                        priority))
                .withName("LEDs.bounce");
    }

    /* Wave */

    public static Command wave(
            Section section,
            Color c1,
            Color c2,
            double cycleLength,
            double duration,
            int priority) {
        return runLEDPattern(() -> leds.wave(section, c1, c2, cycleLength, duration, priority))
                .withName("LEDs.wave");
    }

    /* Chase */

    public static Command chase(double timeout) {
        return runLEDPattern(() -> leds.chase(2)).withTimeout(timeout).withName("LEDs.chase");
    }

    /* Countdown (special) */

    public static Command countdown(double countdownSeconds, int priority) {
        return new FunctionalCommand(
                        leds::setCountdownStartTime,
                        () -> leds.countdown(countdownSeconds, priority),
                        (b) -> {},
                        () -> {
                            return (System.currentTimeMillis() - LEDs.countdownStartTimeMS)
                                    > countdownSeconds * 1000;
                        })
                .ignoringDisable(true) // Run while disabled
                .finallyDo((b) -> leds.resetPriority()) // Reset the Priority when an LED command
                // ends
                .withName("LEDs.runCountdown");
    }

    /** Custom Command Builder */
    public static Command runLEDPattern(Runnable r) {
        // Needs to be Commands.run and not leds.run so it doesn't require led subsystem
        return Commands.run(r) // The the method passed to this method
                .ignoringDisable(true) // Run while disabled
                .finallyDo((b) -> leds.resetPriority()) // Reset priority when an LED command ends
                .withName("LEDs.runLEDCommand"); // Set a default name if one isn't set later
    }
}
