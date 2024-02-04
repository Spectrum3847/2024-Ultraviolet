package frc.robot.vision.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.swerve.commands.SwerveCommands;
import frc.robot.vision.Vision.CommandConfig;
import frc.robot.vision.Vision.VisionConfig;
import frc.spectrumLib.vision.Limelight;

import java.util.function.DoubleSupplier;

public class AlignToVisionTarget extends PIDCommand {
    private static CommandConfig config;
    private static Limelight limelight;
    Command driveCommand;
    DoubleSupplier fwdPositiveSupplier;
    private static double out = 0;
    private double heading = Integer.MIN_VALUE;
    private double horizontalSetpoint;

    /**
     * Creates a new AlignToTarget command that aligns to a vision target (apriltag,
     * retroreflective tape, detector target) on the Field Oriented X-axis.
     *
     * @param commandConfig this config should be created in {@link VisionConfig}
     * @param fwdPositiveSupplier
     * @param offset
     */
    public AlignToVisionTarget(CommandConfig commandConfig, DoubleSupplier fwdPositiveSupplier, double offset) {
        super(
                // The controller that the command will use
                new PIDController(commandConfig.kp, 0, 0),
                // This should return the measurement
                () -> commandConfig.limelight.getHorizontalOffset(),
                // This should return the setpoint (can also be a constant)
                () -> offset,
                // This uses the output
                output -> setOutput(output),
                Robot.swerve);
        config = commandConfig;
        limelight = commandConfig.limelight;
        this.getController().setTolerance(commandConfig.tolerance);
        this.horizontalSetpoint = offset;
        driveCommand =
                SwerveCommands.Drive(
                        fwdPositiveSupplier, // Allows pilot to drive fwd and rev
                        () -> getOutput(), // Moves us center to the tag
                        () -> getSteering(), // Aligns to grid
                        () -> getFieldRelative(), // full velocity
                        () -> true); // Field relative is true
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
        this.setName("AlignToConeOBject");
    }

    public double getSteering() {
        // if customizable heading is set, rotate to that heading
        if (heading != Integer.MIN_VALUE) {
            return Robot.swerve.calculateRotationController(() -> heading);
        }

        // dont set rotation on detector pipelines
        if (config.pipelineIndex > 2) {
            return 0;
        }
        return Robot.swerve.calculateRotationController(() -> Math.PI);
    }

    public boolean getFieldRelative() {
        // We go to RobotPOV to drive straight at the target
        return false;
    }

    public static double getOutput() {
        return out;
    }

    public static void setOutput(double output) {
        out = output;
        if (Math.abs(out) > 1) {
            out = 1 * Math.signum(out);
        }

        out = out * config.maxOutput;
    }

    @Override
    public void initialize() {
        super.initialize();
        out = 0;
        // getLedCommand(tagID).initialize();
        Robot.swerve.resetRotationController();
        driveCommand.initialize();
        limelight.setLimelightPipeline(config.pipelineIndex);
    }

    @Override
    public void execute() {
        super.execute();
        if (this.getController().atSetpoint() || !limelight.targetInView()) {
            out = 0;
        }
        driveCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        // Robot.vision.setLimelightPipeline(VisionConfig.aprilTagPipeline);
        // getLedCommand(tagID).end(interrupted);
    }

    /**
     * Returns true when the command should end. If in auto, automatically end when crosshair gets
     * close to setpoint {@link #error}
     */
    @Override
    public boolean isFinished() {
        if (DriverStation.isAutonomousEnabled()) {
            if (Math.abs(limelight.getHorizontalOffset() - horizontalSetpoint) <= config.error) {
                return true;
            } else {
                return false;
            }
        }
        return false;
    }
}
