package frc.spectrumLib.lasercan;

import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj.DriverStation;

public class LaserCan {
    private au.grapplerobotics.LaserCan lasercan;
    private int id;

    // default constructor
    public LaserCan(int id) {
        this.id = id;
        lasercan = new au.grapplerobotics.LaserCan(id);
        setShortRange();
        setRegionOfInterest(8, 8, 4, 4); // max region
        setTimingBudget(
                au.grapplerobotics.LaserCan.TimingBudget
                        .TIMING_BUDGET_100MS); // Can only set ms to 20, 33, 50, and 100
    }

    public LaserCan(
            int id,
            boolean shortRange,
            int x,
            int y,
            int w,
            int h,
            au.grapplerobotics.LaserCan.TimingBudget timingBudget) {
        lasercan = new au.grapplerobotics.LaserCan(id);
        if (shortRange = true) {
            setShortRange();
        } else {
            setLongRange();
        }
        setRegionOfInterest(x, y, w, h); // max region
        setTimingBudget(timingBudget); // Can only set ms to 20, 33, 50, and 100
    }

    /* Helper methods for constructors */

    public void setShortRange() {
        try {
            lasercan.setRangingMode(au.grapplerobotics.LaserCan.RangingMode.SHORT);
        } catch (ConfigurationFailedException e) {
            logError();
        }
    }

    public void setLongRange() {
        try {
            lasercan.setRangingMode(au.grapplerobotics.LaserCan.RangingMode.LONG);
        } catch (ConfigurationFailedException e) {
            logError();
        }
    }

    public void setRegionOfInterest(int x, int y, int w, int h) {
        try {
            lasercan.setRegionOfInterest(
                    new au.grapplerobotics.LaserCan.RegionOfInterest(x, y, w, h));
        } catch (ConfigurationFailedException e) {
            logError();
        }
    }

    public void setTimingBudget(au.grapplerobotics.LaserCan.TimingBudget timingBudget) {
        try {
            lasercan.setTimingBudget(timingBudget);
        } catch (ConfigurationFailedException e) {
            logError();
        }
    }

    /* Other methods */
    public static void logError() {
        DriverStation.reportWarning("LaserCan: failed to complete operation", false);
    }

    public int getDistance() {
        au.grapplerobotics.LaserCan.Measurement measurement = lasercan.getMeasurement();
        if (measurement != null) {
            if (measurement.status == 0) {
                return measurement.distance_mm;
            } else {
                DriverStation.reportWarning(
                        "LaserCan status went bad: " + measurement.status, false);
                return measurement.distance_mm;
            }
        } else {
            return 0;
        }
    }
}
