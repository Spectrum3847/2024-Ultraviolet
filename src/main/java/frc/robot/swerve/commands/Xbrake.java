package frc.robot.swerve.commands;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.spectrumLib.swerve.Module;
import frc.spectrumLib.swerve.Request;

public class Xbrake implements Request {

    public static Command run() {
        return Robot.swerve.applyRequest(() -> new Xbrake()).withName("Xbrake");
    }

    /** True to use open-loop control while stopped. */
    public boolean IsOpenLoop = false;

    public StatusCode apply(ControlRequestParameters parameters, Module... modulesToApply) {

        SwerveModuleState[] states = new SwerveModuleState[modulesToApply.length];
        for (int i = 0; i < modulesToApply.length; ++i) {
            states[i] = new SwerveModuleState(0, parameters.swervePositions[i].getAngle());
            modulesToApply[i].apply(states[i], IsOpenLoop);
        }
        Robot.swerve.writeSetpoints(states);
        return StatusCode.OK;
    }

    public Xbrake withIsOpenLoop(boolean isOpenLoop) {
        this.IsOpenLoop = isOpenLoop;
        return this;
    }
}
