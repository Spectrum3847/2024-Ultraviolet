package frc.robot.operator;

import frc.robot.RobotTelemetry;
import frc.robot.swerve.commands.SwerveCommands;
import frc.spectrumLib.Gamepad;

public class Operator extends Gamepad {
    public class OperatorConfig {
        public static final String name = "Operator";
        public static final int port = 1;
    }

    public OperatorConfig config;

    /** Create a new Operator with the default name and port. */
    public Operator() {
        super(OperatorConfig.name, OperatorConfig.port);
        config = new OperatorConfig();

        RobotTelemetry.print("Operator Subsystem Initialized: ");
    }

    /** Setup the Buttons for telop mode. */
    /*  A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simualation */
    public void setupTeleopButtons() {
        // controller.a().whileTrue();

        // controller.b().whileTrue();

        // controller.x().toggleOnTrue();

        // controller.y().and(leftBumperOnly()).whileTrue();

        // controller.y().and(rightBumperOnly()).whileTrue();

        // leftXTrigger(ThresholdType.GREATER_THAN, 0).whileTrue();

        controller
                .povUp()
                .and(leftBumperOnly())
                .whileTrue(rumbleCommand(SwerveCommands.reorient(0)));
        controller
                .povLeft()
                .and(leftBumperOnly())
                .whileTrue(rumbleCommand(SwerveCommands.reorient(90)));
        controller
                .povDown()
                .and(leftBumperOnly())
                .whileTrue(rumbleCommand(SwerveCommands.reorient(180)));
        controller
                .povRight()
                .and(leftBumperOnly())
                .whileTrue(rumbleCommand(SwerveCommands.reorient(270)));
    };

    /** Setup the Buttons for Disabled mode. */
    public void setupDisabledButtons() {
        // This is just for training, most robots will have different buttons during disabled
        setupTeleopButtons();
    };

    /** Setup the Buttons for Test mode. */
    public void setupTestButtons() {
        // This is just for training, robots may have different buttons during test
        setupTeleopButtons();
    };
}
