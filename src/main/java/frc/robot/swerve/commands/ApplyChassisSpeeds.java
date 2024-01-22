package frc.robot.swerve.commands;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.spectrumLib.swerve.Module;
import frc.spectrumLib.swerve.Request;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Accepts a generic ChassisSpeeds to apply to the drivetrain. */
public class ApplyChassisSpeeds implements Request {

    public static Command run(Supplier<ChassisSpeeds> speeds, BooleanSupplier isOpenLoop) {
        return Robot.swerve.applyRequest(
                () ->
                        new ApplyChassisSpeeds()
                                .withSpeeds(speeds.get())
                                .withIsOpenLoop(isOpenLoop.getAsBoolean()));
    }

    public static Consumer<ChassisSpeeds> robotRelativeOutput(boolean isOpenLoop) {
        return (speeds) ->
                Robot.swerve.setControlMode(
                        new ApplyChassisSpeeds().withIsOpenLoop(isOpenLoop).withSpeeds(speeds));
    }

    /** The chassis speeds to apply to the drivetrain. */
    public ChassisSpeeds Speeds = new ChassisSpeeds();
    /** The center of rotation to rotate around. */
    public Translation2d CenterOfRotation = new Translation2d(0, 0);
    /** True to use open-loop control while stopped. */
    public boolean IsOpenLoop = false;

    public StatusCode apply(ControlRequestParameters parameters, Module... modulesToApply) {
        SwerveModuleState[] states =
                parameters.kinematics.toSwerveModuleStates(Speeds, CenterOfRotation);
        Robot.swerve.writeSetpoints(states);
        for (int i = 0; i < modulesToApply.length; ++i) {
            modulesToApply[i].apply(states[i], IsOpenLoop);
        }

        return StatusCode.OK;
    }

    public ApplyChassisSpeeds withSpeeds(ChassisSpeeds speeds) {
        this.Speeds = speeds;
        return this;
    }

    public ApplyChassisSpeeds withCenterOfRotation(Translation2d centerOfRotation) {
        this.CenterOfRotation = centerOfRotation;
        return this;
    }

    public ApplyChassisSpeeds withIsOpenLoop(boolean isOpenLoop) {
        this.IsOpenLoop = isOpenLoop;
        return this;
    }
}
