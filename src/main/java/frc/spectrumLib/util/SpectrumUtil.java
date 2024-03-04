package frc.spectrumLib.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class SpectrumUtil {
    /**
     * @param angle degrees
     * @return
     */
    public static double flipAngleIfBlue(double angle) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return 180 - angle;
        }
        return angle;
    }
}
