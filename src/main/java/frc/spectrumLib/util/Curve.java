package frc.spectrumLib.util;

// Spectrum 3847
// Based on Code from FRC# 2363

/**
 * This class and its subclasses remap controller stick inputs according to specifics provided of
 * multiple classes of curves.
 *
 * <p>This is the superclass of several types of curves. It contains most of the code that is used
 * for all types of curves (Linear, Exponential, Spline, and Step).
 *
 * <p>To create a Curve, do not create a <code>Curve</code> object directly, use a subclass of
 * <code>Curve</code> such as <code>ExpCurve</code> instead.
 *
 * @author Justin Babilino
 * @version 0.0.3
 */
public abstract class Curve {
    /** The value added to the curve. */
    private double offset;
    /** The value multiplied to the curve. */
    private double scalar;

    /** The width of the deadband on the curve. */
    private double deadzone;

    /**
     * Calculates and returns a mapped value based on the curve.
     *
     * @param input value to be mapped
     * @return mapped value
     */
    public abstract double calculate(double input);

    /**
     * Returns the value of <code>input</code> mapped to create a deadband of width <code>deadzone
     * </code> in the center of the curve and squishes the rest of the curve to the outside
     * proportionally.
     *
     * @param input the input value to be mapped
     * @return mapped value
     */
    protected double calculateDeadzone(double input) {
        double deadRadius = deadzone / 2.0;
        double val = 0.0;
        if (input > deadRadius) {
            val = (1.0 / (1.0 - deadRadius)) * (input - deadRadius);
        } else if (input < -deadRadius) {
            val = (1.0 / (1.0 - deadRadius)) * (input + deadRadius);
        }
        return val;
    }

    /**
     * Returns the value of <code>input</code> multiplied by the value of <code>scalar</code>.
     *
     * @param input the input value to be mapped
     * @return mapped value
     */
    protected double calculateScalar(double input) {
        double val = input * scalar;
        return val;
    }

    /**
     * Returns the value of <code>input</code> summed with the value of <code>offset</code>.
     *
     * @param input the input value to be mapped
     * @return mapped value
     */
    protected double calculateOffset(double input) {
        double val = input + offset;
        return val;
    }

    /**
     * Returns a set of points of length <code>pointCount</code> on the curve.
     *
     * @param pointCount the amount of points on the curve
     * @return a 2D double array of points on the curve
     */
    public double[][] getCurvePoints(int pointCount) {
        double[][] points = new double[pointCount][2];
        double dx = 2.0 / (pointCount - 1);
        for (int i = 0; i < pointCount; i++) {
            double x = -1.0 + (i * dx);
            points[i][0] = x;
            points[i][1] = calculate(x);
        }
        return points;
    }

    /**
     * Prints the values of a 2D double array of points. The output of this method can be pasted
     * into https://www.desmos.com/calculator to see a visual representation of the <code>Curve
     * </code>.
     *
     * @param points the set of points to be printed
     */
    public void printPoints(double[][] points) {
        System.out.println();
        for (double[] point : points) {
            System.out.print("(" + point[0] + "," + point[1] + ")" + ",");
        }
    }

    /**
     * Prints a set of points on the curve of length <code>pointCount</code>. The output of this
     * method can be pasted into https://www.desmos.com/calculator to see a visual representation of
     * the <code>Curve</code>.
     *
     * @param pointCount the set of points to be printed
     */
    public void printPoints(int pointCount) {
        printPoints(getCurvePoints(pointCount));
    }

    /**
     * Sets the value of <code>offset</code>, the value added to the final curve.
     *
     * @param offset the new value of <code>offset</code>
     */
    public void setOffset(double offset) {
        this.offset = offset;
    }

    /**
     * Sets the value of <code>scalar</code>, the value multiplied to the curve before it is offset.
     *
     * @param scalar the new value of <code>scalar</code>
     */
    public void setScalar(double scalar) {
        this.scalar = scalar;
    }

    /**
     * Sets the value of <code>deadzone</code>, the value for the width in the center of the curve
     * where any input results in an output of <code>0.0</code>.
     *
     * @param deadzone the new value of <code>deadzone</code>
     */
    public void setDeadzone(double deadzone) {
        this.deadzone = Math.abs(deadzone);
    }

    /**
     * Returns the value of <code>offset</code>, the double value added to the final curve.
     *
     * @return the current value of <code>offset</code>
     */
    public double getOffset() {
        return offset;
    }

    /**
     * Returns the value of <code>scalar</code>, the double value multiplied to the curve before it
     * is offset.
     *
     * @return the current value of <code>scalar</code>
     */
    public double getScalar() {
        return scalar;
    }

    /**
     * Returns the value of <code>deadzone</code>, the value for the width in the center of the
     * curve where any input results in an output of <code>0.0</code>
     *
     * @return the current value of <code>deadzone</code>
     */
    public double getDeadzone() {
        return deadzone;
    }
}
