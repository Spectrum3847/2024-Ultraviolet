package frc.robot.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotTelemetry;
import frc.robot.leds.LEDsConfig.Section;
import frc.spectrumLib.leds.SpectrumLEDs;
import java.util.List;

public class LEDs extends SpectrumLEDs {
    public LEDsConfig config;

    public LEDs() {
        super(LEDsConfig.port, LEDsConfig.length);
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
}
