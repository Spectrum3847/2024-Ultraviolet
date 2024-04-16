package frc.robot.vision.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.vision.Vision.CommandConfig;
import frc.spectrumLib.vision.Limelight;

public class DriveToVisionTarget extends PIDCommand {

    private static CommandConfig config;
    private static Limelight limelight;

    private double horizontalOffset = 0; // positive is right (driver POV)

    private static double out = 0;
    private Command alignCommand;

    /**
     * Creates a new DriveToVisionTarget. Aligns to a vision target in both X and Y axes
     * (field-oriented). If used for automation purposes, it is best to give it a timeout as a
     * maximum timeout
     *
     * @param horizontalOffset adjustable offset in the Y axis in case robot isn't completely
     *     aligned. Default value should be 0
     */
    public DriveToVisionTarget(CommandConfig commandConfig, double horizontalOffset) {
        super(
                // The controller that the command will use
                new PIDController(commandConfig.kp, 0, 0),
                // This should return the measurement
                () -> getVerticalOffset(),
                // This should return the setpoint (can also be a constant)
                () -> commandConfig.verticalSetpoint,
                // This uses the output
                output -> {
                    setOutput(output);
                },
                Robot.swerve);
        config = commandConfig;
        limelight = commandConfig.limelight;
        this.horizontalOffset = horizontalOffset;
        alignCommand = getVisionTargetCommand();
        this.getController().setTolerance(commandConfig.tolerance);
        this.setName("DriveToVisionTarget");
    }

    @Override
    public void initialize() {
        super.initialize();
        out = 0;
        alignCommand.initialize();
        limelight.setLimelightPipeline(config.pipelineIndex);
    }

    @Override
    public void execute() {
        super.execute();
        // If we are already closer than the target distance stop driving.
        if (getVerticalOffset() < config.verticalSetpoint || !limelight.targetInView()) {
            out = 0;
        }

        if (getVerticalOffset() > config.verticalMaxView) {
            out = 0;
        }
        alignCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        alignCommand.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // wait for robot to horizontally and vertically aligned before ending
        if (Math.floor(getVerticalOffset()) < config.verticalSetpoint
                && alignCommand.isFinished()) {
            return true;
        }
        if (getVerticalOffset() > config.verticalMaxView) {
            return true;
        }
        return false;
    }

    // If out is > 1 then cap at one, make the robot drive slow and still have bigger kP
    private static void setOutput(double output) {
        out = -1 * output;
        if (Math.abs(out) > 1) {
            out = 1 * Math.signum(out);
        }

        // Multiply the output times how fast we want the max speed to drive is.
        out = out * config.maxOutput;

        // If we are going to slow increase the speed to the min
        // if (Math.abs(out) < minOutput) {
        //     out = minOutput * Math.signum(out);
        // }
    }

    // We are driving negative and our Ty values are getting smaller so that works.
    public static double getOutput() {
        return -out; // camera is on back of robot
    }

    // Align and drive while we are doing this, pass output to it so it can drive forward.
    public Command getVisionTargetCommand() {
        return new AlignToVisionTarget(config.alignCommand, () -> getOutput(), horizontalOffset);
    }

    // Returns the measurement from the Limelight
    private static double getVerticalOffset() {
        return limelight.getVerticalOffset();
    }
}
