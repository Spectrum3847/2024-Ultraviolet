package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.spectrumLib.swerve.config.SwerveConfig;

/**
 * Uses a profiled PID Controller to quickly turn the robot to a specified angle. Once the robot is
 * within a certain tolerance of the goal angle, a PID controller is used to hold the robot at that
 * angle.
 */
public class RotationController {
    Swerve swerve;
    SwerveConfig config;
    ProfiledPIDController controller;
    PIDController holdController;
    Constraints constraints;

    public RotationController(Swerve swerve) {
        this.swerve = swerve;
        config = swerve.config;
        constraints = new Constraints(config.maxAngularVelocity, config.maxAngularAcceleration);
        controller =
                new ProfiledPIDController(
                        config.kPRotationController,
                        config.kIRotationController,
                        config.kDRotationController,
                        constraints);

        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(Math.PI / 180);

        // These are currently magic number and need to be put into SwerveConfig
        holdController = new PIDController(10.5, 3, 0);

        holdController.enableContinuousInput(-Math.PI, Math.PI);
        holdController.setTolerance(Math.PI / 180);
    }

    public double calculate(double goalRadians) {
        double calculatedValue =
                controller.calculate(swerve.getRotation().getRadians(), goalRadians);
        if (atSetpoint()) {
            return calculateHold(goalRadians);
        } else {
            return calculatedValue;
        }
    }

    public double calculateHold(double goalRadians) {
        double calculatedValue =
                holdController.calculate(swerve.getRotation().getRadians(), goalRadians);
        return calculatedValue;
    }

    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    public boolean atHoldSetpoint() {
        return holdController.atSetpoint();
    }

    public void reset() {
        controller.reset(swerve.getRotation().getRadians());
        holdController.reset();
    }
}
