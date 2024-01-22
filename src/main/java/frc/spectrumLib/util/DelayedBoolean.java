package frc.spectrumLib.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

/** An iterative boolean latch that delays the transition from false to true. */
public class DelayedBoolean {
    private boolean mLastStatus;
    private double mTransitionTimestamp;
    private final double mDelay; // seconds

    /**
     * Create a new DelayedBoolean with a custom delay in seconds.
     *
     * @param delay seconds
     */
    public DelayedBoolean(double delay) {
        mTransitionTimestamp = Timer.getFPGATimestamp();
        mLastStatus = false;
        mDelay = delay;
    }

    public boolean update(boolean status) {
        double timestamp = Timer.getFPGATimestamp();
        boolean result = false;

        if (status && !mLastStatus) {
            mTransitionTimestamp = timestamp;
        }

        // If we are still true and we have transitioned.
        if (status && (timestamp - mTransitionTimestamp > mDelay)) {
            result = true;
        }

        mLastStatus = status;
        return result;
    }

    /**
     * Creates a Trigger with a delayed activation based on a given condition. The delay starts once
     * the condition becomes true. If the condition becomes false again before the delay expires,
     * the behavior depends on the 'continuous' flag.
     *
     * @param condition A BooleanSupplier representing the condition that initiates the delay. The
     *     trigger's delay starts when this condition returns true.
     * @param delay The time, in seconds, to wait after the condition turns true before activating
     *     the trigger.
     * @param continuous If true, the delay timer resets if the condition becomes false before the
     *     delay has passed, requiring the condition to remain continuously true for the entire
     *     delay duration. If false, the timer does not reset if the condition becomes false,
     *     allowing the trigger to activate if the condition becomes true again within the original
     *     delay period.
     * @return A new Trigger object that activates after the specified delay. This trigger is useful
     *     in various one-time actions but should used carefully when in conjunction with
     *     Trigger.whileTrue() as it will never return to false.
     */
    public static Trigger triggerWithDelay(
            BooleanSupplier condition, double delay, boolean continuous) {
        BooleanSupplier delayedSupplier =
                new BooleanSupplier() {
                    private double trueSince = Integer.MAX_VALUE;
                    private boolean delayPassed = false;

                    @Override
                    public boolean getAsBoolean() {
                        if (trueSince == Integer.MAX_VALUE) {
                            if (condition.getAsBoolean()) {
                                // start timer
                                trueSince = Timer.getFPGATimestamp();
                                delayPassed = Timer.getFPGATimestamp() - trueSince >= delay;
                                return delayPassed; // in case delay is 0, instantly return true
                                // when original condition switches
                            }
                        } else {
                            delayPassed = Timer.getFPGATimestamp() - trueSince >= delay;
                            // if delay hasn't passed yet and original condition is no longer true,
                            // reset timer
                            if (continuous && !delayPassed && !condition.getAsBoolean()) {
                                trueSince = Integer.MAX_VALUE;
                                return false;
                            }
                            return delayPassed;
                        }

                        return false;
                    }
                };

        return new Trigger(delayedSupplier);
    }
}
