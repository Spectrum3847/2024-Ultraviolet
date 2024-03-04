package frc.robot.operator;

import frc.robot.RobotCommands;
import frc.robot.RobotTelemetry;
import frc.robot.leds.LEDsCommands;
import frc.robot.mechanisms.climber.ClimberCommands;
import frc.robot.mechanisms.elevator.ElevatorCommands;
import frc.robot.mechanisms.launcher.LauncherCommands;
import frc.robot.vision.VisionCommands;
import frc.spectrumLib.gamepads.Gamepad;

public class Operator extends Gamepad {
    public class OperatorConfig {
        public static final String name = "Operator";
        public static final int port = 1;
        /**
         * in order to run a PS5 controller, you must use DS4Windows to emulate a XBOX controller as
         * well and move the controller to emulatedPS5Port
         */
        public static final boolean isXbox = true;

        public static final int emulatedPS5Port = 5;
    }

    public OperatorConfig config;

    /** Create a new Operator with the default name and port. */
    public Operator() {
        super(
                OperatorConfig.name,
                OperatorConfig.port,
                OperatorConfig.isXbox,
                OperatorConfig.emulatedPS5Port);
        config = new OperatorConfig();

        RobotTelemetry.print("Operator Subsystem Initialized: ");
    }

    /** Setup the Buttons for telop mode. */
    /*  A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simualation */
    public void setupTeleopButtons() {
        controller.a().and(noBumpers()).whileTrue(RobotCommands.smartIntake());
        controller.a().and(leftBumperOnly()).whileTrue(LauncherCommands.stopMotors());

        controller.b().and(noBumpers()).whileTrue(RobotCommands.amp());
        controller.b().and(leftBumperOnly()).onTrue(RobotCommands.subwooferShot());

        controller.y().and(noBumpers()).whileTrue(RobotCommands.eject());
        controller.y().and(leftBumperOnly()).onTrue(RobotCommands.onDemandLaunching());

        controller.x().and(noBumpers()).whileTrue(ElevatorCommands.amp());
        controller.x().and(leftBumperOnly()).whileTrue(ElevatorCommands.home());

        controller.leftBumper().and(rightTriggerOnly()).onTrue(RobotCommands.podiumShot());

        rightStick().and(leftBumperOnly()).whileTrue(OperatorCommands.manualPivot());
        leftStick().and(leftBumperOnly()).whileTrue(OperatorCommands.manualElevator());

        bothBumpers()
                .whileTrue(
                        LEDsCommands.solidGreenLED()
                                .alongWith(VisionCommands.resetPoseWithVision()));

        controller.upDpad().and(leftBumperOnly()).whileTrue(RobotCommands.topClimb());
        controller.downDpad().and(leftBumperOnly()).whileTrue(ClimberCommands.midClimb());
        controller.leftDpad().and(leftBumperOnly()).whileTrue(ElevatorCommands.fullExtend());
        controller.rightDpad().and(leftBumperOnly()).whileTrue(ClimberCommands.botClimb());
    };

    /** Setup the Buttons for Disabled mode. */
    public void setupDisabledButtons() {

        controller.b().toggleOnTrue(RobotCommands.coastModeMechanisms());
    };

    /** Setup the Buttons for Test mode. */
    public void setupTestButtons() {
        // This is just for training, robots may have different buttons during test
        setupTeleopButtons();
    };
}
