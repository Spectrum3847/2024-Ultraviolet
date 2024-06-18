package frc.robot.leds;

import edu.wpi.first.wpilibj.util.Color;

public class LEDsConfig {
    public static final int port = 0;
    public static final int length = 29;
    public static final double strobeFastDuration = 0.1;
    public static final double strobeSlowDuration = 0.2;
    public static final double breathDuration = 1.0;
    public static final double rainbowCycleLength = 25.0;
    public static final double rainbowDuration = 0.25;
    public static final double waveExponent = 0.4;
    public static final double waveFastCycleLength = 25.0;
    public static final double waveFastDuration = 0.25;
    public static final double waveSlowCycleLength = 25.0;
    public static final double waveSlowDuration = 3.0;
    public static final double waveAllianceCycleLength = 15.0;
    public static final double waveAllianceDuration = 2.0;
    public static final double autoFadeTime = 2.5;
    public static final double autoFadeMaxTime = 5.0;

    public static final Color SPECTRUM_COLOR = new Color(130, 103, 185);

    public enum Section {
        FULL,
        HALF_LOW,
        HALF_HIGH,
        QUARTER_LOW,
        QUARTER_HIGH;

        public int start() {
            switch (this) {
                case FULL:
                    return 0;
                case HALF_LOW:
                    return 0;
                case HALF_HIGH:
                    return length - (length / 2);
                case QUARTER_LOW:
                    return 0;
                case QUARTER_HIGH:
                    return length - (length / 4);
                default:
                    return 0;
            }
        }

        public int end() {
            switch (this) {
                case FULL:
                    return length;
                case HALF_LOW:
                    return length / 2;
                case HALF_HIGH:
                    return length;
                case QUARTER_LOW:
                    return length / 4;
                case QUARTER_HIGH:
                    return length;
                default:
                    return 0;
            }
        }
    }
}
