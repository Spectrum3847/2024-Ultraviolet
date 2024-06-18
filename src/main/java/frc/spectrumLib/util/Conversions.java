package frc.spectrumLib.util;

import java.util.function.DoubleSupplier;

public class Conversions {

    /**
     * Converts revolutions per minute (RPM) to revolutions per second (RPS).
     *
     * @param rpm the value in RPM to be converted
     * @return the converted value in RPS
     */
    public static double RPMtoRPS(double rpm) {
        return rpm / 60;
    }

    /**
     * Converts revolutions per minute (RPM) to revolutions per second (RPS).
     *
     * @param rpm the value in RPM to be converted
     * @return the converted value in RPS
     */
    public static Double RPMtoRPS(DoubleSupplier rpm) {
        return rpm.getAsDouble() / 60;
    }

    /**
     * Converts revolutions per second (RPS) to revolutions per minute (RPM).
     *
     * @param rps the value in RPS to be converted
     * @return the converted value in RPM
     */
    public static double RPStoRPM(double rps) {
        return rps * 60;
    }
}
