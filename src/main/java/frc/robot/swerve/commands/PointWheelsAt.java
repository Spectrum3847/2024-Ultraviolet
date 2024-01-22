package frc.robot.swerve.commands;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.spectrumLib.swerve.Module;
import frc.spectrumLib.swerve.Request;

public class PointWheelsAt implements Request {

    public static Command run(Rotation2d moduleDirection, boolean isOpenLoop) {
        return Robot.swerve.applyRequest(
                () ->
                        new PointWheelsAt()
                                .withModuleDirection(moduleDirection)
                                .withIsOpenLoop(isOpenLoop));
    }
    /**
     * The direction to point the modules toward. This direction is still optimized to what the
     * module was previously at.
     */
    public Rotation2d ModuleDirection = new Rotation2d();
    /** True to use open-loop control while stopped. */
    public boolean IsOpenLoop = false;

    public StatusCode apply(ControlRequestParameters parameters, Module... modulesToApply) {
        SwerveModuleState[] states = new SwerveModuleState[modulesToApply.length];
        for (int i = 0; i < modulesToApply.length; ++i) {
            states[i] = new SwerveModuleState(0, ModuleDirection);
            modulesToApply[i].apply(states[i], IsOpenLoop);
        }

        Robot.swerve.writeSetpoints(states);

        return StatusCode.OK;
    }

    public PointWheelsAt withModuleDirection(Rotation2d moduleDirection) {
        this.ModuleDirection = moduleDirection;
        return this;
    }

    public PointWheelsAt withIsOpenLoop(boolean isOpenLoop) {
        this.IsOpenLoop = isOpenLoop;
        return this;
    }
}
