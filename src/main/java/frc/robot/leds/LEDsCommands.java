package frc.robot.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.leds.LEDsConfig.Section;

public class LEDsCommands {
    private static LEDs leds = Robot.leds;

    public static void setupDefaultCommand() {
        leds.setDefaultCommand(defaultCommand());
    }

    /** Specific Commands */
    public static Command defaultCommand() {
        return leds.run(
                        () -> {
                            rainbow(Section.FULL, LEDsConfig.length / 2, 2, 0).execute();
                        })
                .ignoringDisable(true)
                .withName("LEDs.default");
    }

    public static Command solidPurpleLED() {
        return LEDsCommands.solid(Section.HALF_HIGH, Color.kPurple, 2)
                .withName("LEDs.solidPurpleLED");
    }

    public static Command strobeOrangeLED() {
        return LEDsCommands.strobe(Section.QUARTER_HIGH, Color.kOrange, 0.5, 2)
                .withName("LEDs.strobeOrangeLED");
    }

    public static Command breathBlueLED() {
        return LEDsCommands.breath(Section.FULL, Color.kBlue, Color.kBlack, 1, 4)
                .withName("LEDs.breathBlueLED");
    }

    /** Common Commands */
    public static Command runLEDPattern(Runnable r) {
        // Needs to be Commands.run and not leds.run so it doesn't require led subsystem
        return Commands.run(r) // The the method passed to this method
                .ignoringDisable(true) // Run while disabled
                .finallyDo((b) -> leds.resetPriority()) // Reset the Priority when an LED command
                // ends
                .withName("LEDs.runLEDCommand"); // Set a default name if one isn't set later
    }

    public static Command solid(Color color, int priority) {
        return solid(Section.FULL, color, priority);
    }

    public static Command solid(Section section, Color color, int priority) {
        return runLEDPattern(() -> leds.solid(section, color, priority)).withName("LEDs.solid");
    }

    public static Command solid(double percent, Color color, int priority) {
        return runLEDPattern(() -> leds.solid(percent, color, priority)).withName("LEDs.solid");
    }

    public static Command strobe(Color color, int priority) {
        return strobe(Section.FULL, color, 0.5, priority);
    }

    public static Command strobe(Section section, Color color, double duration, int priority) {
        return runLEDPattern(() -> leds.strobe(section, color, duration, priority))
                .withName("LEDs.strobe");
    }

    public static Command breath(Color c1, Color c2, int priority) {
        return breath(Section.FULL, c1, c2, 1, priority);
    }

    public static Command breath(
            Section section, Color c1, Color c2, double duration, int priority) {
        return runLEDPattern(() -> leds.breath(section, c1, c2, duration, priority))
                .withName("LEDs.breath");
    }

    public static Command rainbow(int priority) {
        return rainbow(Section.FULL, LEDsConfig.length, 1, priority);
    }

    public static Command rainbow(
            Section section, double cycleLength, double duration, int priority) {
        return runLEDPattern(() -> leds.rainbow(section, cycleLength, duration, priority))
                .withName("LEDs.rainbow");
    }

    public static Command wave(Color c1, Color c2, int priority) {
        return wave(Section.FULL, c1, c2, LEDsConfig.length, 1, priority);
    }

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
}
