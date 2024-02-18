package frc.robot.operator;

import frc.robot.RobotCommands;
import frc.robot.RobotTelemetry;
import frc.robot.mechanisms.elevator.ElevatorCommands;
import frc.robot.mechanisms.launcher.LauncherCommands;
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
        // dummy driver practice
        controller.a().and(noBumpers()).whileTrue(RobotCommands.dummyIntake());
        controller.a().and(leftBumperOnly()).whileTrue(LauncherCommands.stopMotors());

        controller.b().and(noBumpers()).whileTrue(RobotCommands.feedToAmp());
        controller.y().and(leftBumperOnly()).onTrue(RobotCommands.subwooferReady());

        controller.y().and(noBumpers()).whileTrue(RobotCommands.eject());
        controller.b().and(leftBumperOnly()).onTrue(RobotCommands.onDemandLaunching());

        controller.x().and(noBumpers()).whileTrue(ElevatorCommands.amp());
        controller.x().and(leftBumperOnly()).whileTrue(ElevatorCommands.home());

        // manual output commands (map joystick to raw -1 to 1 output on motor): manualAmpTrap,
        // manualClimber, manualElevator, manualFeeder, manualIntake, manualPivot, manualLauncher

        // controller.a().whileTrue();

        // controller.b().whileTrue();

        // controller.x().toggleOnTrue();

        // controller.y().and(leftBumperOnly()).whileTrue();

        // controller.y().and(rightBumperOnly()).whileTrue();

        // leftXTrigger(ThresholdType.GREATER_THAN, 0).whileTrue();

        // controller.rightBumper().whileTrue(RobotCommands.feedToAmp());
        // controller.povUp().and(leftBumperOnly()).whileTrue(ClimberCommands.topClimb());
        // controller.povDown().and(leftBumperOnly()).whileTrue(ClimberCommands.midClimb());
        // controller.povLeft().and(leftBumperOnly()).whileTrue(ElevatorCommands.fullExtend());
        // controller.povRight().and(leftBumperOnly()).whileTrue(ClimberCommands.botClimb());
    };

    /** Setup the Buttons for Disabled mode. */
    public void setupDisabledButtons() {
        // This is just for training, most robots will have different buttons during disabled

        controller.b().toggleOnTrue(RobotCommands.coastModeMechanisms());
    };

    /** Setup the Buttons for Test mode. */
    public void setupTestButtons() {
        // This is just for training, robots may have different buttons during test
        setupTeleopButtons();
    };
}
