package frc.robot.pilot;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotTelemetry;
import frc.robot.mechanisms.amptrap.AmpTrapCommands;
import frc.robot.mechanisms.climber.ClimberCommands;
import frc.robot.mechanisms.elevator.ElevatorCommands;
import frc.robot.mechanisms.feeder.FeederCommands;
import frc.robot.mechanisms.intake.IntakeCommands;
import frc.robot.swerve.commands.SwerveCommands;
import frc.spectrumLib.Gamepad;
import frc.spectrumLib.util.ExpCurve;

public class Pilot extends Gamepad {
    public class PilotConfig {
        public static final String name = "Pilot";
        public static final int port = 0;

        public static final double slowModeScalor = 0.5;

        public final double leftStickDeadzone = 0.1;
        public final double leftStickExp = 2.0;
        public final double leftStickScalor = Robot.swerve.config.maxVelocity;

        public final double triggersDeadzone = 0.1;
        public final double triggersExp = 2.0;
        public final double triggersScalor = Robot.swerve.config.maxAngularVelocity;
        public final double rotationScalor = 0.8;
    }

    public PilotConfig config;
    private boolean isSlowMode = false;
    private boolean isFieldOriented = true;
    private ExpCurve LeftStickCurve;
    private ExpCurve TriggersCurve;

    /** Create a new Pilot with the default name and port. */
    public Pilot() {
        super(PilotConfig.name, PilotConfig.port);
        config = new PilotConfig();

        // Curve objects that we use to configure the controller axis ojbects
        LeftStickCurve =
                new ExpCurve(
                        config.leftStickExp, 0, config.leftStickScalor, config.leftStickDeadzone);
        TriggersCurve =
                new ExpCurve(config.triggersExp, 0, config.triggersScalor, config.triggersDeadzone);

        RobotTelemetry.print("Pilot Subsystem Initialized: ");
    }

    /** Setup the Buttons for telop mode. */
    /*  A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simualation */
    public void setupTeleopButtons() {
        // manual output commands (map joystick to raw -1 to 1 output on motor): manualAmpTrap,
        // manualClimber, manualElevator, manualFeeder, manualIntake, manualPivot, manualLauncher

        controller.y().and(noBumpers()).whileTrue(ClimberCommands.raise());

        controller.a().and(noBumpers()).whileTrue(ClimberCommands.lower());

        controller.b().and(leftBumperOnly()).whileTrue(ClimberCommands.fullExtend());

        // controller.a().and(leftBumperOnly()).whileTrue(ClimberCommands.home());

        controller.y().and(leftBumperOnly()).whileTrue(ElevatorCommands.fullExtend());

        controller.a().and(leftBumperOnly()).whileTrue(ElevatorCommands.home());

        // controller.x().and(noBumpers()).whileTrue(LauncherCommands.runOnDemandVelocity());

        controller
                .x()
                .and(leftBumperOnly())
                .whileTrue(FeederCommands.testBack().alongWith(AmpTrapCommands.testForward()));

        controller.rightBumper().whileTrue(IntakeCommands.intake());

        rightStick()
                .and(leftBumperOnly())
                .whileTrue(ClimberCommands.manualCommand(() -> controller.getRightY()));
        // controller.b().and(leftBumperOnly()).whileTrue(PivotCommands.halfScore());

        // controller.a().and(leftBumperOnly()).whileTrue(ElevatorCommands.amp());

        // controller
        //         .x()
        //         .and(leftBumperOnly())
        //         .whileTrue(
        //                 AmpTrapCommands.testForward()
        //                         .alongWith(
        //                                 FeederCommands.testBack(),
        //                                 IntakeCommands.runTestin(),
        //                                 ElevatorCommands.home()));

        // controller
        //         .y()
        //         .and(leftBumperOnly())
        //         .whileTrue(
        //                 AmpTrapCommands.testReverse()
        //                         .alongWith(FeederCommands.testBack(), IntakeCommands.eject()));

        // controller.y().and(leftBumperOnly()).whileTrue(LauncherCommands.runVelocityTestin());

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

        // Use the pilot drive if we are manually steering the robot
        controller
                .rightTrigger(config.triggersDeadzone)
                .or(controller.leftTrigger(config.triggersDeadzone))
                .whileTrue(PilotCommands.pilotDrive());

        // Use the right stick to set a cardinal direction to aim at
        // rightXTrigger(ThresholdType.ABS_GREATER_THAN, 0.5)
        //         .and(rightYTrigger(ThresholdType.ABS_GREATER_THAN, 0.5))
        //         .whileTrue(PilotCommands.stickSteerDrive());
    };

    /** Setup the Buttons for Disabled mode. */
    public void setupDisabledButtons() {
        // This is just for training, most robots will have different buttons during disabled
        // setupTeleopButtons();

        rightStick()
                .and(leftBumperOnly())
                .whileTrue(
                        new RunCommand(() -> System.out.println(-controller.getRightY()))
                                .ignoringDisable(true));
    };

    /** Setup the Buttons for Test mode. */
    public void setupTestButtons() {
        // This is just for training, robots may have different buttons during test
        setupTeleopButtons();
    };

    public void setMaxVelocity(double maxVelocity) {
        LeftStickCurve.setScalar(maxVelocity);
    }

    public void setMaxRotationalVelocity(double maxRotationalVelocity) {
        TriggersCurve.setScalar(maxRotationalVelocity);
    }

    public void setSlowMode(boolean isSlowMode) {
        this.isSlowMode = isSlowMode;
    }

    public void setFieldOriented(boolean isFieldOriented) {
        this.isFieldOriented = isFieldOriented;
    }

    public boolean getFieldOriented() {
        return isFieldOriented;
    }

    // Positive is forward, up on the left stick is positive
    // Applies Expontial Curve, Deadzone, and Slow Mode toggle
    public double getDriveFwdPositive() {
        double fwdPositive = LeftStickCurve.calculate(-1 * controller.getLeftY());
        if (isSlowMode) {
            fwdPositive *= Math.abs(PilotConfig.slowModeScalor);
        }
        return fwdPositive;
    }

    // Positive is left, left on the left stick is positive
    // Applies Expontial Curve, Deadzone, and Slow Mode toggle
    public double getDriveLeftPositive() {
        double leftPositive = -1 * LeftStickCurve.calculate(controller.getLeftX());
        if (isSlowMode) {
            leftPositive *= Math.abs(PilotConfig.slowModeScalor);
        }
        return leftPositive;
    }

    // Positive is counter-clockwise, left Trigger is positive
    // Applies Expontial Curve, Deadzone, and Slow Mode toggle
    public double getDriveCCWPositive() {
        double ccwPositive = TriggersCurve.calculate(getTwist());
        if (isSlowMode) {
            ccwPositive *= Math.abs(PilotConfig.slowModeScalor);
        } else {
            ccwPositive *= config.rotationScalor;
        }
        return ccwPositive;
    }

    public Trigger rightStick() {
        return new Trigger(
                () -> {
                    return controller.getRightX() != 0 || controller.getRightY() != 0;
                });
    }
}
