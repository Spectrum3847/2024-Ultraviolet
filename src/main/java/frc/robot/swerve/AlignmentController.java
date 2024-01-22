package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.spectrumLib.swerve.config.SwerveConfig;

/**
 * Uses a profiled PID Controller to quickly align the robot to a specified position. Once the robot
 * is within a certain tolerance of the goal position, a PID controller is used to hold the robot at
 * that position.
 */
public class AlignmentController {
    Swerve swerve;
    SwerveConfig config;
    ProfiledPIDController controller;
    PIDController holdController;
    Constraints constraints;
    double tolerance = 0.01;
    double kP;
    double kI;
    double kD;

    public AlignmentController(Swerve swerve, double kP, double kI, double kD) {
        this.swerve = swerve;
        config = swerve.config;
        constraints = new Constraints(config.maxVelocity, config.maxAccel);
        controller =
                new ProfiledPIDController(
                        config.kPRotationController,
                        config.kIRotationController,
                        config.kDRotationController,
                        constraints);

        controller.setTolerance(0.01); // 1 CM tolerance

        // These are currently magic number and need to be put into SwerveConfig
        holdController = new PIDController(10.5, 3, 0);

        holdController.setTolerance(tolerance);
    }

    public AlignmentController withConstraints(double maxVelocity, double maxAccel) {
        constraints = new Constraints(maxVelocity, maxAccel);
        controller.setConstraints(constraints);
        return this;
    }

    public AlignmentController withTolerance(double tolerance) {
        this.tolerance = tolerance;
        controller.setTolerance(tolerance);
        holdController.setTolerance(tolerance);
        return this;
    }

    public double calculate(double currentPosition, double goalMeters) {
        double calculatedValue = controller.calculate(currentPosition, goalMeters);
        if (atSetpoint()) {
            return calculateHold(currentPosition, goalMeters);
        } else {
            return calculatedValue;
        }
    }

    public double calculateHold(double currentPostion, double goalRadians) {
        double calculatedValue = holdController.calculate(currentPostion, goalRadians);
        return calculatedValue;
    }

    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    public boolean atHoldSetpoint() {
        return holdController.atSetpoint();
    }

    public void reset(double currentPose) {
        controller.reset(currentPose);
        holdController.reset();
    }
}
