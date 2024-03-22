package frc.robot.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotTelemetry;
import frc.robot.leds.LEDsConfig.Section;
import frc.spectrumLib.leds.SpectrumLEDs;
import java.util.List;

public class LEDs extends SpectrumLEDs {
    public LEDsConfig config;

    public static long countdownStartTimeMS = System.currentTimeMillis();
    public static double strobeCounter = 0;
    public static boolean coastModeLED = false;
    public static boolean launchReadyLED = false;

    public LEDs() {
        super(LEDsConfig.port, LEDsConfig.length * 2);
        config = new LEDsConfig();

        RobotTelemetry.print("LEDs Subsystem Initialized: ");
    }

    // LED Patterns
    // Many of these were borrowed from 6328-2023 code
    public void solid(Section section, Color color, int priority) {
        if (getUpdate()) {
            if (color != null) {
                for (int i = section.start(); i < section.end(); i++) {
                    setLED(i, color, priority);
                }
            }
        }
    }

    /** @param endLeds how many leds on each side of the ends of strip should be lit */
    public void limitedSolid(int endLeds, Color color, int priority) {
        if (getUpdate()) {
            if (color != null) {
                for (int i = Section.FULL.start(); i < Section.FULL.end(); i++) {
                    if (i <= endLeds || i >= Section.FULL.end() - endLeds) {
                        setLED(i, color, priority);
                    }
                }
            }
        }
    }

    /**
     * Sets a percentage of the LEDs to a color
     *
     * @param percent The percentage of LEDs to set
     * @param color The color to set the LEDs to
     */
    public void solid(double percent, Color color, int priority) {
        if (getUpdate()) {
            for (int i = 0;
                    i < MathUtil.clamp(LEDsConfig.length * percent, 0, LEDsConfig.length);
                    i++) {
                setLED(i, color, priority);
            }
        }
    }

    /**
     * Set a section of the LEDs to strobe
     *
     * @param section The section of the LEDs to strobe
     * @param color The color to strobe
     * @param duration The duration of the strobe
     */
    public void strobe(Section section, Color color, double duration, int priority) {
        if (getUpdate()) {
            boolean on = ((getLEDTime() % duration) / duration) > 0.5;
            solid(section, on ? color : Color.kBlack, priority);
        }
    }

    /** @param frequency in robot cycles */
    public void customStrobe(Section section, Color color, double frequency, int priority) {
        if (getUpdate()) {
            boolean on = false;
            strobeCounter++;
            if (strobeCounter >= frequency * 2) {
                restartStrobeCounter();
            }
            if (strobeCounter < frequency) {
                on = true;
            }
            solid(section, on ? color : Color.kBlack, priority);
        }
    }

    /** @param endLeds how many leds on each side of the ends of strip should be lit */
    public void limitedStrobe(int endLeds, Color color, double duration, int priority) {
        if (getUpdate()) {
            boolean on = ((getLEDTime() % duration) / duration) > 0.5;
            limitedSolid(endLeds, on ? color : Color.kBlack, priority);
        }
    }

    public void breath(Section section, Color c1, Color c2, double duration, int priority) {
        breath(section, c1, c2, duration, getLEDTime(), priority);
    }

    public void breath(
            Section section, Color c1, Color c2, double duration, double timestamp, int priority) {
        if (getUpdate()) {
            double x =
                    ((timestamp % LEDsConfig.breathDuration) / LEDsConfig.breathDuration)
                            * 2.0
                            * Math.PI;
            double ratio = (Math.sin(x) + 1.0) / 2.0;
            double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
            double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
            double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
            solid(section, new Color(red, green, blue), priority);
        }
    }

    /**
     * Sets the LEDs to a rainbow pattern
     *
     * @param section The section of the LEDs to set
     * @param cycleLength The length of the rainbow cycle in LEDs
     * @param duration The duration of the rainbow
     */
    public void rainbow(Section section, double cycleLength, double duration, int priority) {
        if (getUpdate()) {
            double x = (1 - ((getLEDTime() / duration) % 1.0)) * 180.0;
            double xDiffPerLed = 180.0 / cycleLength;
            for (int i = section.start(); i < section.end(); i++) {
                x += xDiffPerLed;
                x %= 180.0;
                setHSV(i, (int) x, 255, 255, priority);
            }
        }
    }

    /**
     * Sets the LEDs to a wave pattern
     *
     * @param section The section of the LEDs to set
     * @param c1 The first color of the wave
     * @param c2 The second color of the wave
     * @param cycleLength The length of the wave cycle in LEDs
     * @param duration The duration of the wave
     */
    public void wave(
            Section section,
            Color c1,
            Color c2,
            double cycleLength,
            double duration,
            int priority) {
        if (getUpdate()) {
            double x = (1 - ((getLEDTime() % duration) / duration)) * 2.0 * Math.PI;
            double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
            for (int i = section.start(); i < section.end(); i++) {
                x += xDiffPerLed;

                double ratio = (Math.pow(Math.sin(x), LEDsConfig.waveExponent) + 1.0) / 2.0;
                if (Double.isNaN(ratio)) {
                    ratio = (-Math.pow(Math.sin(x + Math.PI), LEDsConfig.waveExponent) + 1.0) / 2.0;
                }
                if (Double.isNaN(ratio)) {
                    ratio = 0.5;
                }
                double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
                double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
                double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
                setLED(i, new Color(red, green, blue), priority);
            }
        }
    }

    public void stripes(
            Section section, List<Color> colors, int length, double duration, int priority) {
        if (getUpdate()) {
            int offset = (int) (getLEDTime() % duration / duration * length * colors.size());
            for (int i = section.start(); i < section.end(); i++) {
                int colorIndex =
                        (int) (Math.floor((double) (i - offset) / length) + colors.size())
                                % colors.size();
                colorIndex = colors.size() - 1 - colorIndex;
                setLED(i, colors.get(colorIndex), priority);
            }
        }
    }

    public void defaultPattern() {
        final int defaultPriority = 0;
        if (getUpdate()) {
            // DS check takes priority
            if (!DriverStation.isDSAttached()) {
                strobe(Section.FULL, Color.kOrangeRed, 1, defaultPriority);
            } else if (DriverStation.isDisabled()) {
                // solid(Section.FULL, Color.kWhite, defaultPriority);
                ombre(Section.FULL, LEDsConfig.SPECTRUM_COLOR, Color.kWhite, defaultPriority);
            } else if (DriverStation.isAutonomousEnabled()) {
                solid(Section.FULL, Color.kBlack, defaultPriority);
            } else if (DriverStation.isTestEnabled()) {
                bounce(
                        Section.FULL,
                        Color.kWhite,
                        new Color(205, 205, 205),
                        new Color(155, 155, 155),
                        new Color(130, 103, 185),
                        0.58,
                        defaultPriority);
            } else {
                bounce(
                        Section.FULL,
                        new Color(130, 103, 185),
                        new Color(80, 53, 135),
                        new Color(30, 3, 85),
                        Color.kBlack,
                        0.58,
                        0);
            }
        }
    }

    public void bounce(
            Section section,
            Color primaryColor,
            Color secondaryColor,
            Color tertiaryColor,
            Color fillColor,
            double speed,
            int priority) {
        if (getUpdate()) {
            long currentTime = System.currentTimeMillis();
            double cycleTime = 1000 / speed; // Convert speed to milliseconds for the entire cycle
            double phase = (currentTime % cycleTime) / cycleTime; // Phase of the cycle from 0 to 1

            // Determine direction and position based on the phase
            boolean backwards = phase > 0.5;
            double position = backwards ? 2 * (1 - phase) : 2 * phase;
            int ledPosition =
                    (int) (position * (section.end() - section.start())) + section.start();

            // Set LEDs based on the current position and direction
            for (int i = section.start(); i < section.end(); i++) {
                if (i == ledPosition) {
                    setLED(i, primaryColor, priority); // Main LED
                } else if (i == ledPosition - 1 || i == ledPosition + 1) {
                    setLED(i, secondaryColor, priority); // Immediate neighbors
                } else if (i == ledPosition - 2 || i == ledPosition + 2) {
                    setLED(i, tertiaryColor, priority); // Next neighbors
                } else {
                    setLED(i, fillColor, priority); // Turn off other LEDs
                }
            }
        }
    }

    public void staticGradient(Section section, Color startColor, Color endColor, int priority) {
        if (getUpdate()) {
            int totalLEDs = section.end() - section.start();
            for (int i = section.start(); i < section.end(); i++) {
                double ratio = (double) (i - section.start()) / totalLEDs;

                // Interpolate the red, green, and blue components separately using double precision
                double red = (startColor.red * (1 - ratio)) + (endColor.red * ratio);
                double green = (startColor.green * (1 - ratio)) + (endColor.green * ratio);
                double blue = (startColor.blue * (1 - ratio)) + (endColor.blue * ratio);

                // Create a new color for the current LED
                Color currentColor = new Color(red, green, blue);

                // Set the color of the current LED
                setLED(i, currentColor, priority);
            }
        }
    }

    public void ombre(Section section, Color startColor, Color endColor, int priority) {
        if (getUpdate()) {
            long currentTime = System.currentTimeMillis();
            // The speed factor here determines how quickly the ombre moves along the strip
            double phaseShift =
                    (currentTime / 1000.0)
                            * 0.58 // this is the speed, the higher the number the faster the ombre
                            // moves
                            % 1.0; // Modulo 1 to keep the phase within [0, 1]

            int totalLEDs = section.end() - section.start();
            for (int i = section.start(); i < section.end(); i++) {
                // Adjust ratio to include the phaseShift, causing the ombre to move
                double ratio =
                        (double) (i - section.start() + totalLEDs * phaseShift)
                                / totalLEDs
                                % 1.0; // Modulo 1 to ensure the ratio loops within [0, 1]

                // Interpolate the red, green, and blue components separately using double precision
                double red = (startColor.red * (1 - ratio)) + (endColor.red * ratio);
                double green = (startColor.green * (1 - ratio)) + (endColor.green * ratio);
                double blue = (startColor.blue * (1 - ratio)) + (endColor.blue * ratio);

                // Create a new color for the current LED
                Color currentColor = new Color(red, green, blue);

                // Set the color of the current LED
                setLED(i, currentColor, priority);
            }
        }
    }

    /**
     * Countdown LED sequence. This method {@code must} have its countdownStartTimeMS set {@code
     * only once} before the sequence is run. (Command only sequence)
     *
     * @param durationInSeconds The total duration of the countdown in seconds.
     * @param priority The priority of this LED pattern.
     */
    public void countdown(double durationInSeconds, int priority) {
        if (getUpdate()) {
            long currentTimeMillis = System.currentTimeMillis();
            // Assuming the start time is stored somewhere, calculate elapsed time
            double elapsedTimeInSeconds = (currentTimeMillis - countdownStartTimeMS) / 1000.0;

            // Calculate the progress of the countdown
            double progress = elapsedTimeInSeconds / durationInSeconds;

            // Calculate the number of LEDs to turn off based on the progress
            int ledsToTurnOff = (int) (LEDsConfig.length * progress);

            // Calculate the color transition from yellow to red based on the progress
            // Yellow (255, 255, 0) to Red (255, 0, 0)
            int red = 255; // Red component stays at 255
            int green = (int) (255 * (1 - progress)); // Green component decreases to 0
            Color countdownColor = new Color(red, green, 0);

            // Update the LEDs from the end of the strip towards the beginning
            for (int i = LEDsConfig.length - 1; i >= 0; i--) {
                if (LEDsConfig.length - i <= ledsToTurnOff) {
                    // Turn off the LEDs progressively
                    setLED(i, Color.kBlack, priority);
                } else {
                    // Set the remaining LEDs to the countdown color
                    setLED(i, countdownColor, priority);
                }
            }

            // If the countdown is complete, ensure all LEDs are turned off
            if (progress >= 1.0) {
                for (int i = 0; i < LEDsConfig.length; i++) {
                    setLED(i, Color.kBlack, priority);
                }
            }
        }
    }

    public void chase(int priority) {
        if (getUpdate()) {
            long currentTimeMillis = System.currentTimeMillis();
            // Define the duration of each color phase (red/blue) in milliseconds
            double phaseDuration = 150; // Adjust for faster or slower transitions

            // Calculate the current phase based on time
            int phase = (int) ((currentTimeMillis / phaseDuration) % 4); // 4 phases

            Color chaseColor;
            switch (phase) {
                case 0: // Red phase
                    chaseColor = Color.kRed;
                    break;
                case 2: // Blue phase
                    chaseColor = Color.kBlue;
                    break;
                default: // Off phases
                    chaseColor = Color.kBlack;
                    break;
            }

            // Set the color of the entire strip based on the current phase
            for (int i = 0; i < LEDsConfig.length; i++) {
                setLED(i, chaseColor, priority);
            }

            // To add a moving effect
            if (phase == 0 || phase == 2) {
                int moveDuration =
                        LEDsConfig.length / 4; // Number of LEDs to cover for the moving effect
                for (int i = 0; i < moveDuration; i++) {
                    int index =
                            (int)
                                    ((currentTimeMillis / 100)
                                            % LEDsConfig.length); // Moving index based on time
                    index =
                            (phase == 0)
                                    ? index
                                    : LEDsConfig.length
                                            - 1
                                            - index; // Reverse direction for blue phase
                    setLED(
                            index,
                            chaseColor,
                            priority + 1); // Slightly higher priority to override the static color
                }
            }
        }
    }

    /* Helper */
    public void setCountdownStartTime() {
        countdownStartTimeMS = System.currentTimeMillis();
    }

    public void restartStrobeCounter() {
        strobeCounter = 0;
    }

    public static void turnOnCoastLEDs() {
        coastModeLED = true;
    }

    public static void turnOffCoastLEDs() {
        coastModeLED = false;
    }

    public static void turnOnLaunchLEDs() {
        launchReadyLED = true;
    }

    public static void turnOffLaunchLEDs() {
        launchReadyLED = false;
    }
}
