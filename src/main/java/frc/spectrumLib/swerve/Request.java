package frc.spectrumLib.swerve;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * Container for all the Swerve Requests. Use this to find all applicable swerve drive requests.
 *
 * <p>This is also an interface common to all swerve drive control requests that allow the request
 * to calculate the state to apply to the modules.
 */
public interface Request {

    /*
     * Contains everything the control requests need to calculate the module state
     */
    public class ControlRequestParameters {
        public SwerveDriveKinematics kinematics;
        public Pose2d currentPose;
        public double timestamp;
        public Translation2d[] swervePositions;
        public double updatePeriod;
    }

    public StatusCode apply(ControlRequestParameters parameters, Module... modulesToApply);

    /**
     * Does nothing to the swerve module state. This is the default state of a newly created swerve
     * drive mechanism.
     */
    public class Idle implements Request {

        /** True to use open-loop control while stopped. */
        public boolean IsOpenLoop = false;

        public StatusCode apply(ControlRequestParameters parameters, Module... modulesToApply) {

            return StatusCode.OK;
        }

        public Idle withIsOpenLoop(boolean isOpenLoop) {
            this.IsOpenLoop = isOpenLoop;
            return this;
        }
    }
}
