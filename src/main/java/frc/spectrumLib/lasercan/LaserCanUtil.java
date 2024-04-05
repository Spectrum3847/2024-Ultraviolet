package frc.spectrumLib.lasercan;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DriverStation;

public class LaserCanUtil {
    private LaserCan lasercan;
    private int id;

    // default constructor
    public LaserCanUtil(int id) {
        this.id = id;
        lasercan = new LaserCan(id);
        setShortRange();
        setRegionOfInterest(8, 8, 4, 4); // max region
        setTimingBudget(
                LaserCan.TimingBudget
                        .TIMING_BUDGET_100MS); // Can only set ms to 20, 33, 50, and 100
    }

    public LaserCanUtil(
            int id,
            boolean shortRange,
            int x,
            int y,
            int w,
            int h,
            LaserCan.TimingBudget timingBudget) {
        lasercan = new LaserCan(id);
        if (shortRange = true) {
            setShortRange();
        } else {
            setLongRange();
        }
        setRegionOfInterest(x, y, w, h); // max region
        setTimingBudget(timingBudget); // Can only set ms to 20, 33, 50, and 100
    }

    /* Internal Lasercan methods */
    public boolean hasNote() {
        if (getDistance() < 0) {
            return false;
        }
        return getDistance() < 300;
    }

    public boolean intakedNote() {
        if (getDistance() < 0) {
            return false;
        }
        return getDistance() < 400;
    }

    public boolean midNote() {
        if (getDistance() < 0) {
            return false;
        }
        return Math.abs(getDistance()) - 10 <= 0;
    }

    public boolean bigMidNote() {
        if (getDistance() < 0) {
            return false;
        }
        return Math.abs(getDistance() - 50) <= 0;
    }

    public boolean endNote() {
        if (getDistance() < 0) {
            return false;
        }
        return getDistance() > 250;
    }

    public boolean closeNote() {
        if (getDistance() < 0) {
            return false;
        }
        return getDistance() < 60;
    }

    public boolean validDistance() {
        return getDistance() >= 0;
    }

    /* Helper methods for constructors */

    public void setShortRange() {
        try {
            lasercan.setRangingMode(LaserCan.RangingMode.SHORT);
        } catch (ConfigurationFailedException e) {
            logError();
        }
    }

    public void setLongRange() {
        try {
            lasercan.setRangingMode(LaserCan.RangingMode.LONG);
        } catch (ConfigurationFailedException e) {
            logError();
        }
    }

    public void setRegionOfInterest(int x, int y, int w, int h) {
        try {
            lasercan.setRegionOfInterest(new LaserCan.RegionOfInterest(x, y, w, h));
        } catch (ConfigurationFailedException e) {
            logError();
        }
    }

    public void setTimingBudget(LaserCan.TimingBudget timingBudget) {
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
        LaserCan.Measurement measurement = lasercan.getMeasurement();
        if (measurement != null) {
            if (measurement.status == 0) {
                return measurement.distance_mm;
            } else {
                if (measurement.status != 2) {
                    DriverStation.reportWarning(
                            "LaserCan #" + id + " status went bad: " + measurement.status, false);
                }
                return measurement.distance_mm;
            }
        } else {
            return -1000;
        }
    }
}
